/*
 * Terminal shell for STM32F103C8T6
 * Supports: local UART1 (USB-serial) + remote UART2 (ESP8266 WiFi)
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <board.h>
#include "shell.h"
#include "../oled/OLED.h"
#include "../wifi_config.h"
#include "../esp8266/esp8266.h"
#include "../hardware/pca9685.h"
#include "../hardware/relay.h"
#include "../hardware/pushrod.h"
#include "../hardware/cprod.h"
#include "../hardware/lift.h"
#include "../servo_motion.h"
#include "../preset_store.h"
#include "../thread_store.h"

extern void shell_led_on(void);
extern void shell_led_off(void);
extern void shell_led_blink(void);
extern char g_esp8266_ip[17];
extern volatile int _wifi_connecting;

/* -------- command table -------- */
struct shell_cmd {
    const char *name;
    const char *desc;
    void (*func)(int argc, char **argv);
};

static void shell_exec(char *line);

/* -------- output helper -------- */
static rt_device_t _remote_dev = RT_NULL;

/* ring buffer: save non-CIPSEND data read by remote_send */
static char _save_buf[512];
static int  _save_len = 0;

/* remote input: per-link buffers (5 links max) */
static char _remote_buf[5][256];
static int  _remote_len[5];

/* per-link output + active link (-1=serial/local) */
static char _out_buf[5][768];
static int  _out_len[5];
static int  _active_link = -1;
static volatile int _tx_busy = 0;
static int  _banner_link = -1;
static int  _banner_tick = 0;
static void writer_post(int raw, int link, const char *data, int len);

static struct rt_mutex _out_mutex;

static void shell_write(const char *str)
{
    rt_kprintf("%s", str);  /* serial console sees everything */
    int link = _active_link;
    if (link >= 0 && link < 5) {
        rt_mutex_take(&_out_mutex, RT_WAITING_FOREVER);
        int slen = rt_strlen(str);
        if (_out_len[link] + slen < 768 - 1) {
            rt_memcpy(_out_buf[link] + _out_len[link], str, slen);
            _out_len[link] += slen;
            _out_buf[link][_out_len[link]] = '\0';
        }
        rt_mutex_release(&_out_mutex);
    }
}

/* public output hook for the script runner thread (thread_store.c) */
void shell_put(const char *str)
{
    shell_write(str);
}

/* case-insensitive keyword equality */
static int _ci_eq(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca >= 'a' && ca <= 'z') ca -= 32;
        if (cb >= 'a' && cb <= 'z') cb -= 32;
        if (ca != cb) return 0;
        a++; b++;
    }
    return *a == *b;
}

/* strict non-negative integer parse: every char must be a digit.
 * Stops "sw"/"rlx"/"s12x" from atoi()→0 masquerading as s0/rl1. */
static int _atoi_strict(const char *s, int *out)
{
    if (!s || !*s) return 0;
    int v = 0;
    for (const char *p = s; *p; p++) {
        if (*p < '0' || *p > '9') return 0;
        v = v * 10 + (*p - '0');
        if (v > 100000) return 0;   /* overflow guard */
    }
    *out = v;
    return 1;
}

static int _in_edit = 0;   /* 1 = editing a thread script */

/* ===================================================================
 *  Writer-thread IPC — one writer, per-link output
 *  =================================================================== */
static struct rt_mutex     _remote_mutex;
static struct rt_semaphore _tx_sem;
static char _tx_buf[768];
static int  _tx_len = 0;
static int  _tx_raw = 0;
static int  _tx_link = 0;

static void writer_post(int raw, int link, const char *data, int len)
{
    if (!_remote_dev || len <= 0 || len > (int)sizeof(_tx_buf)) return;
    _tx_busy = 1;
    rt_memcpy(_tx_buf, data, len);
    _tx_len  = len;
    _tx_raw  = raw;
    _tx_link = link;
    rt_sem_release(&_tx_sem);
}

static void remote_send_raw(int link, const char *data, int len)
{
    if (!_remote_dev || len == 0) return;
    char cmd[32];
    rt_snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d,%d\r\n", link, len);
    rt_device_write(_remote_dev, 0, cmd, rt_strlen(cmd));
    rt_thread_mdelay(200);
    {
        rt_device_write(_remote_dev, 0, data, len);
        int dt = rt_tick_get();
        while (rt_tick_get() - dt < rt_tick_from_millisecond(300)) {
            char dc;
            if (rt_device_read(_remote_dev, 0, &dc, 1) == 1) {
                if (dc != '\0' && _save_len < (int)sizeof(_save_buf) - 1)
                    _save_buf[_save_len++] = dc;
            } else rt_thread_mdelay(5);
        }
    }
}

static void remote_send(int link, const char *str)
{
    int len = rt_strlen(str);
    if (!_remote_dev || len == 0) return;
    char cmd[32];
    rt_snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d,%d\r\n", link, len);
    rt_device_write(_remote_dev, 0, cmd, rt_strlen(cmd));
    rt_thread_mdelay(200);
    {
        rt_device_write(_remote_dev, 0, str, len);
        int dt = rt_tick_get();
        while (rt_tick_get() - dt < rt_tick_from_millisecond(300)) {
            char dc;
            if (rt_device_read(_remote_dev, 0, &dc, 1) == 1) {
                if (dc != '\0' && _save_len < (int)sizeof(_save_buf) - 1)
                    _save_buf[_save_len++] = dc;
            } else rt_thread_mdelay(5);
        }
    }
}

static void remote_flush(void)
{
    int link = _active_link;
    if (link < 0 || link > 4) { rt_kprintf(SHELL_PROMPT); return; }
    int plen = rt_strlen(SHELL_PROMPT);

    while (_tx_busy) rt_thread_mdelay(5);  /* wait for previous send to finish */

    rt_mutex_take(&_out_mutex, RT_WAITING_FOREVER);
    if (_out_len[link] + plen < 768 - 1) {
        rt_memcpy(_out_buf[link] + _out_len[link], SHELL_PROMPT, plen);
        _out_len[link] += plen;
        _out_buf[link][_out_len[link]] = '\0';
    }
    if (_remote_dev && _out_len[link] > 0)
        writer_post(0, link, _out_buf[link], _out_len[link] + 1);
    _out_len[link] = 0;
    rt_mutex_release(&_out_mutex);

    rt_kprintf("%s", SHELL_PROMPT);
}

/* -------- commands -------- */
static void cmd_help(int argc, char **argv)
{
    shell_write("\r\n");
    shell_write("  === STM32F407 Shell ===\r\n");
    shell_write("  help          - show help\r\n");
    shell_write("  led  on/off   - turn LED on/off\r\n");
    shell_write("  info          - system info\r\n");
    shell_write("  reset         - reset MCU\r\n");
    shell_write("  ota [serial|wifi] - OTA update\r\n");
    shell_write("  who           - show connections\r\n");
    shell_write("  oled <L> <T>  - show text on OLED\r\n");
    shell_write("  wifi show/save/delete/exc/mode\r\n");
    shell_write("  S<n> <deg> [ms]  - servo n rotate (0-180); ms=0..10000 timed\r\n");
    shell_write("  S<n> OFF         - servo n off (no PWM)\r\n");
    shell_write("  S<n> P<m>        - servo n apply preset m (1-8)\r\n");
    shell_write("  RL<n> ON|OFF     - relay n (1 only; 2-8 = actuators)\r\n");
    shell_write("  MP <UP|DN> [ms]  - push rod; MP END / MP OFF / MP STOP / MP P<m>\r\n");
    shell_write("  CP <UP|DN> [ms]  - CP push rod; CP OFF / CP STOP / CP P<m>\r\n");
    shell_write("  LIFT <UP|DN> [ms|end] - lift platform; LIFT OFF / LIFT STOP\r\n");
    shell_write("  set S<n> P<m> <deg|OFF> [ms]  - save servo preset\r\n");
    shell_write("  set MP P<m> <UP|DN|OFF>       - save pushrod preset\r\n");
    shell_write("  set CP P<m> <UP|DN|OFF>       - save CP pushrod preset\r\n");
    shell_write("  show S<n> | show MP | show CP | show <name> - view presets/script\r\n");
    shell_write("  Run <name>       - run thread script (Thread1..16)\r\n");
    shell_write("  edit <name>      - edit thread (insert/edit/delete/quit)\r\n");
    shell_write("  stop             - stop everything + reset actuators\r\n");
    shell_write("  hardware      - full hardware help\r\n");
    shell_write("\r\n");
}

static void cmd_led(int argc, char **argv)
{
    if (argc < 2) { shell_write("Usage: led <on|off|blink>\r\n"); return; }
    if      (rt_strcmp(argv[1], "on")   ==0) { shell_led_on();    shell_write("LED ON\r\n"); }
    else if (rt_strcmp(argv[1], "off")  ==0) { shell_led_off();   shell_write("LED OFF\r\n"); }
    else if (rt_strcmp(argv[1], "blink")==0) { shell_led_blink(); shell_write("LED blink\r\n"); }
    else shell_write("Unknown\r\n");
}

static void cmd_info(int argc, char **argv)
{
    shell_write("\r\n  STM32F407VET6  Cortex-M4  168MHz  512KB Flash  128KB RAM\r\n\r\n");
}

static void cmd_reset(int argc, char **argv)
{
    shell_write("reset...\r\n");
    rt_thread_mdelay(100);
    rt_hw_cpu_reset();
}

static void cmd_ota(int argc, char **argv)
{
    uint32_t flag = 0x4F544101;  /* serial */

    if (argc >= 2 && rt_strcmp(argv[1], "wifi") == 0) {
        flag = 0x4F544102;
        /* Bootloader will query IP itself via AT+CIPSTATUS */
    } else if (argc >= 2 && rt_strcmp(argv[1], "serial") == 0) {
        flag = 0x4F544101;
    }

    if (flag == 0x4F544102)
        shell_write("Rebooting to WiFi OTA mode...\r\n");
    else
        shell_write("Rebooting to Serial OTA mode...\r\n");
    OLED_Clear(); OLED_ShowString(1, 1, "Bootloader OTA..");

    {
        /* Store OTA flag in RTC backup register (survives reset, lost on power-cycle) */
        rt_base_t lv = rt_hw_interrupt_disable();
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_DBP;
        RTC->BKP0R = flag;
        rt_hw_interrupt_enable(lv);
    }

    rt_thread_mdelay(100);
    rt_hw_cpu_reset();
}

static void cmd_who(int argc, char **argv)
{
    shell_write("\r\n  Local:  UART1\r\n");
    shell_write(_remote_dev ? "  Remote: ESP8266\r\n\r\n" : "  Remote: --\r\n\r\n");
}

static void cmd_oled(int argc, char **argv)
{
    if (argc < 2) {
        shell_write("Usage: oled <line> <text>   - show text on OLED line 2-4\r\n");
        shell_write("       oled clear          - clear OLED screen\r\n");
        return;
    }

    if (rt_strcmp(argv[1], "clear") == 0) {
        OLED_Clear();
        if (g_esp8266_ip[0]) OLED_ShowString(1, 1, g_esp8266_ip);
        shell_write("OLED cleared\r\n");
        return;
    }

    int line = 0;
    if (argv[1][0] >= '2' && argv[1][0] <= '4' && argv[1][1] == '\0')
        line = argv[1][0] - '0';
    else {
        shell_write("Line must be 2-4 (line 1 is reserved)\r\n");
        return;
    }

    if (argc < 3) {
        shell_write("Usage: oled <line> <text>\r\n");
        return;
    }

    OLED_ShowString(line, 1, "                ");
    OLED_ShowString(line, 1, argv[2]);

    char buf[64];
    rt_snprintf(buf, sizeof(buf), "OLED line %d: %s\r\n", line, argv[2]);
    shell_write(buf);
}

static void cmd_wifi(int argc, char **argv)
{
    if (argc < 2) {
        shell_write("Usage:\r\n");
        shell_write("  wifi show                  - list saved networks\r\n");
        shell_write("  wifi save <ssid> <pass> [p]- save network at priority p\r\n");
        shell_write("  wifi delete <priority>     - delete network\r\n");
        shell_write("  wifi mode <sta|ap> [ssid] [pass]\r\n");
        return;
    }

    if (rt_strcmp(argv[1], "show") == 0) {
        wifi_config_show(shell_write);
    }
    else if (rt_strcmp(argv[1], "save") == 0) {
        if (argc < 4) {
            shell_write("Usage: wifi save <ssid> <password> [priority]\r\n");
            return;
        }
        int prio = (argc >= 5) ? atoi(argv[4]) : 0;
        if (wifi_config_save(argv[2], argv[3], prio) == 0)
            shell_write("WiFi saved.\r\n");
        else
            shell_write("Error: flash full or write failed.\r\n");
    }
    else if (rt_strcmp(argv[1], "delete") == 0) {
        if (argc < 3) {
            shell_write("Usage: wifi delete <priority>\r\n");
            return;
        }
        int prio = atoi(argv[2]);
        if (wifi_config_delete(prio) == 0)
            shell_write("WiFi deleted.\r\n");
        else
            shell_write("Error: priority not found.\r\n");
    }
    else if (rt_strcmp(argv[1], "exc") == 0) {
        if (argc < 4) {
            shell_write("Usage: wifi exc <prio1> <prio2>\r\n");
            return;
        }
        int p1 = atoi(argv[2]);
        int p2 = atoi(argv[3]);
        if (wifi_config_swap(p1, p2) == 0)
            shell_write("WiFi priorities swapped.\r\n");
        else
            shell_write("Error: priority not found.\r\n");
    }
    else if (rt_strcmp(argv[1], "mode") == 0) {
        if (argc < 3) {
            shell_write("Usage: wifi mode <sta|ap> [ap_ssid] [ap_pass]\r\n");
            return;
        }
        int mode = WIFI_MODE_STA;
        if (rt_strcmp(argv[2], "ap") == 0)        mode = WIFI_MODE_AP;
        else if (rt_strcmp(argv[2], "sta") == 0)  mode = WIFI_MODE_STA;
        else {
            shell_write("mode must be 'sta' or 'ap'\r\n");
            return;
        }

        const char *ssid = (argc >= 4) ? argv[3] : g_ap_ssid;
        const char *pass = (argc >= 5) ? argv[4] : g_ap_pass;

        if (wifi_mode_save(mode, ssid, pass) == 0) {
            shell_write(mode == WIFI_MODE_AP
                        ? "AP mode saved. Rebooting...\r\n"
                        : "STA mode saved. Rebooting...\r\n");
            rt_thread_mdelay(100);
            rt_hw_cpu_reset();
        } else {
            shell_write("Error: failed to save mode to flash.\r\n");
        }
    }
    else {
        shell_write("Unknown wifi sub-command.\r\n");
    }
}

/* servo: S<n> <deg|OFF|P<m>> [ms], n=0..15 */
static void cmd_servo(int argc, char **argv, int num)
{
    if (argc < 2) {
        shell_write("Usage: S<n> <deg|OFF|P<m>> [ms]  (deg 0-180, 90=center)\r\n");
        return;
    }

    if (_ci_eq(argv[1], "OFF")) {
        servo_motion_off((uint8_t)num);
        char buf[48];
        rt_snprintf(buf, sizeof(buf), "Servo %d off (no PWM)\r\n", num);
        shell_write(buf);
        return;
    }

    if (argv[1][0] == 'P' || argv[1][0] == 'p') {
        int m;
        if (!_atoi_strict(argv[1] + 1, &m) || m < 1 || m > PRESET_POSITIONS) {
            shell_write("Preset must be p1..p8\r\n");
            return;
        }
        int d = preset_apply_servo((uint8_t)num, (uint8_t)m);
        char buf[48];
        rt_snprintf(buf, sizeof(buf), "Servo %d -> preset p%d (%dms)\r\n", num, m, d);
        shell_write(buf);
        return;
    }

    int deg;
    if (!_atoi_strict(argv[1], &deg)) {
        shell_write("Deg must be 0..180 (or OFF / P<m>)\r\n");
        return;
    }
    if (deg > PCA9685_SERVO_MAX_DEG) deg = PCA9685_SERVO_MAX_DEG;

    int ms = 0;
    if (argc >= 3) {
        if (!_atoi_strict(argv[2], &ms)) {
            shell_write("Duration ms must be 0..10000\r\n");
            return;
        }
        if (ms > SERVO_MAX_DUR_MS) ms = SERVO_MAX_DUR_MS;
    }

    servo_motion_move((uint8_t)num, (uint16_t)deg, (uint16_t)ms);

    char buf[64];
    if (ms > 0)
        rt_snprintf(buf, sizeof(buf), "Servo %d -> %d deg over %dms\r\n", num, deg, ms);
    else
        rt_snprintf(buf, sizeof(buf), "Servo %d -> %d deg\r\n", num, deg);
    shell_write(buf);
}

/* relay: RL<n> ON|OFF, n=1 (only relay 1 is free; 2-8 reserved for actuators) */
static void cmd_relay(int argc, char **argv, int num)
{
    int on;
    if (num == 2 || num == 3 || num == 4 || num == 5 || num == 6 || num == 7 || num == 8) {
        shell_write("RL2-8 are reserved (2-3=LIFT, 4-5=CP, 6-8=MP).\r\n");
        shell_write("Use: LIFT <UP|DN> [ms|end] | MP ... | CP ...\r\n");
        return;
    }
    if (argc < 2) {
        shell_write("Usage: RL<n> ON|OFF\r\n");
        return;
    }
    if      (rt_strcmp(argv[1], "ON")  == 0 || rt_strcmp(argv[1], "on")  == 0) on = 1;
    else if (rt_strcmp(argv[1], "OFF") == 0 || rt_strcmp(argv[1], "off") == 0) on = 0;
    else {
        shell_write("Usage: RL<n> ON|OFF\r\n");
        return;
    }

    relay_set(num, on);

    char buf[48];
    rt_snprintf(buf, sizeof(buf), "Relay %d %s\r\n", num, on ? "ON" : "OFF");
    shell_write(buf);
}

/* push rod: MP <UP|DN> [ms] | MP OFF | MP STOP | MP P<m> — relays 6-8 H-bridge */
static void cmd_pushrod(int argc, char **argv)
{
    if (argc < 2) {
        shell_write("Usage: MP <UP|DN> [ms] | MP END | MP OFF | MP STOP | MP P<m>\r\n");
        shell_write("       ms - optional; timed move 0..20000 ms, omit = continuous\r\n");
        shell_write("       END - move DN until the limit switch (PC0) triggers\r\n");
        return;
    }

    if (_ci_eq(argv[1], "OFF")) {
        pushrod_off();
        shell_write("Pushrod off\r\n");
        return;
    }

    if (_ci_eq(argv[1], "STOP")) {
        pushrod_stop();
        shell_write("Pushrod stop\r\n");
        return;
    }

    if (_ci_eq(argv[1], "END")) {
        if (pushrod_move_end() != 0) {
            shell_write("Pushrod busy (a timed move is running; 'MP STOP' first)\r\n");
            return;
        }
        shell_write("Pushrod DN until limit switch...\r\n");
        return;
    }

    if (argv[1][0] == 'P' || argv[1][0] == 'p') {
        int m;
        if (!_atoi_strict(argv[1] + 1, &m) || m < 1 || m > PRESET_POSITIONS) {
            shell_write("Preset must be p1..p8\r\n");
            return;
        }
        preset_apply_pushrod((uint8_t)m);
        char buf[48];
        rt_snprintf(buf, sizeof(buf), "Pushrod -> preset p%d\r\n", m);
        shell_write(buf);
        return;
    }

    int dir;
    if      (_ci_eq(argv[1], "UP")) dir = PUSHROD_UP;
    else if (_ci_eq(argv[1], "DN")) dir = PUSHROD_DOWN;
    else {
        shell_write("Usage: MP <UP|DN> [ms]\r\n");
        return;
    }

    char buf[64];

    /* continuous: just set the relay state (no timing, never "busy") */
    if (argc < 3) {
        pushrod_set(dir);
        rt_snprintf(buf, sizeof(buf), "Pushrod %s (continuous)\r\n",
                    (dir == PUSHROD_UP) ? "UP" : "DN");
        shell_write(buf);
        return;
    }

    /* timed move */
    int m = atoi(argv[2]);
    if (m < 0) m = 0;
    if (m > PUSHROD_MAX_MS) m = PUSHROD_MAX_MS;

    if (pushrod_move(dir, (uint32_t)m) != 0) {
        shell_write("Pushrod busy (a timed move is running; 'MP STOP' first)\r\n");
        return;
    }

    rt_snprintf(buf, sizeof(buf), "Pushrod %s %dms\r\n",
                (dir == PUSHROD_UP) ? "UP" : "DN", m);
    shell_write(buf);
}

/* CP push rod: CP <UP|DN> [ms] | CP OFF | CP STOP | CP P<m> — relays 4-5 */
static void cmd_cprod(int argc, char **argv)
{
    if (argc < 2) {
        shell_write("Usage: CP <UP|DN> [ms] | CP OFF | CP STOP | CP P<m>\r\n");
        shell_write("       ms - optional; timed move 0..20000 ms, omit = continuous\r\n");
        return;
    }

    if (_ci_eq(argv[1], "OFF")) {
        cprod_off();
        shell_write("CP rod off\r\n");
        return;
    }

    if (_ci_eq(argv[1], "STOP")) {
        cprod_stop();
        shell_write("CP rod stop\r\n");
        return;
    }

    if (argv[1][0] == 'P' || argv[1][0] == 'p') {
        int m;
        if (!_atoi_strict(argv[1] + 1, &m) || m < 1 || m > PRESET_POSITIONS) {
            shell_write("Preset must be p1..p8\r\n");
            return;
        }
        preset_apply_cprod((uint8_t)m);
        char buf[48];
        rt_snprintf(buf, sizeof(buf), "CP rod -> preset p%d\r\n", m);
        shell_write(buf);
        return;
    }

    int dir;
    if      (_ci_eq(argv[1], "UP")) dir = CPROD_UP;
    else if (_ci_eq(argv[1], "DN")) dir = CPROD_DOWN;
    else {
        shell_write("Usage: CP <UP|DN> [ms]\r\n");
        return;
    }

    char buf[64];

    /* continuous: just set the relay state (no timing, never "busy") */
    if (argc < 3) {
        cprod_set(dir);
        rt_snprintf(buf, sizeof(buf), "CP rod %s (continuous)\r\n",
                    (dir == CPROD_UP) ? "UP" : "DN");
        shell_write(buf);
        return;
    }

    /* timed move */
    int m = atoi(argv[2]);
    if (m < 0) m = 0;
    if (m > CPROD_MAX_MS) m = CPROD_MAX_MS;

    if (cprod_move(dir, (uint32_t)m) != 0) {
        shell_write("CP rod busy ('CP STOP' first)\r\n");
        return;
    }

    rt_snprintf(buf, sizeof(buf), "CP rod %s %dms\r\n",
                (dir == CPROD_UP) ? "UP" : "DN", m);
    shell_write(buf);
}

/* lift platform: LIFT <UP|DN> [ms|end] | LIFT OFF | LIFT STOP — relays 2-3.
 * Timed moves stop early on the direction's limit switch; "end" (or omitting
 * ms) moves until that limit switch triggers. */
static void cmd_lift(int argc, char **argv)
{
    if (argc < 2) {
        shell_write("Usage: LIFT <UP|DN> [ms|end] | LIFT OFF | LIFT STOP\r\n");
        shell_write("       ms - timed move (stops early on limit); end/omit = until limit\r\n");
        return;
    }

    if (_ci_eq(argv[1], "OFF")) {
        lift_off();
        shell_write("Lift off\r\n");
        return;
    }

    if (_ci_eq(argv[1], "STOP")) {
        lift_stop();
        shell_write("Lift stop\r\n");
        return;
    }

    int dir;
    if      (_ci_eq(argv[1], "UP")) dir = LIFT_UP;
    else if (_ci_eq(argv[1], "DN")) dir = LIFT_DN;
    else {
        shell_write("Usage: LIFT <UP|DN> [ms|end]\r\n");
        return;
    }

    /* end / no-arg -> move until the direction's limit switch */
    if (argc < 3 || _ci_eq(argv[2], "END")) {
        if (lift_move_end(dir) != 0) {
            shell_write("Lift busy (a move is running; 'LIFT STOP' first)\r\n");
            return;
        }
        shell_write((dir == LIFT_UP) ? "Lift UP until limit...\r\n" : "Lift DN until limit...\r\n");
        return;
    }

    int ms;
    if (!_atoi_strict(argv[2], &ms) || ms < 0) {
        shell_write("Usage: LIFT <UP|DN> [ms|end]\r\n");
        return;
    }
    if (ms > LIFT_MAX_MS) ms = LIFT_MAX_MS;

    if (lift_move(dir, (uint32_t)ms) != 0) {
        shell_write("Lift busy (a move is running; 'LIFT STOP' first)\r\n");
        return;
    }

    char buf[48];
    rt_snprintf(buf, sizeof(buf), "Lift %s %dms\r\n",
                (dir == LIFT_UP) ? "UP" : "DN", ms);
    shell_write(buf);
}

/* set S<n> P<m> <deg|OFF> [ms]  |  set MP P<m> <UP|DN|OFF>  |  set CP P<m> <UP|DN|OFF> */
static void cmd_set(int argc, char **argv)
{
    if (argc < 4) {
        shell_write("Usage: set S<n> P<m> <deg|OFF> [ms]\r\n");
        shell_write("       set MP P<m> <UP|DN|OFF>\r\n");
        shell_write("       set CP P<m> <UP|DN|OFF>\r\n");
        return;
    }

    if (argv[1][0] == 'S' || argv[1][0] == 's') {
        int n;
        if (!_atoi_strict(argv[1] + 1, &n) || n < 0 || n > 15) { shell_write("Servo must be S0..S15\r\n"); return; }
        if (argv[2][0] != 'P' && argv[2][0] != 'p') {
            shell_write("Usage: set S<n> P<m> <deg|OFF> [ms]\r\n"); return;
        }
        int m;
        if (!_atoi_strict(argv[2] + 1, &m) || m < 1 || m > PRESET_POSITIONS) { shell_write("Preset must be p1..p8\r\n"); return; }

        uint16_t deg, ms = 0;
        if (_ci_eq(argv[3], "OFF")) deg = PRESET_SERVO_OFF;
        else {
            deg = (uint16_t)atoi(argv[3]);
            if (deg > 180) deg = 180;
        }
        if (argc >= 5) {
            ms = (uint16_t)atoi(argv[4]);
            if (ms > SERVO_MAX_DUR_MS) ms = SERVO_MAX_DUR_MS;
        }

        preset_set_servo((uint8_t)n, (uint8_t)m, deg, ms);
        char buf[64];
        if (deg == PRESET_SERVO_OFF)
            rt_snprintf(buf, sizeof(buf), "Saved S%d p%d = OFF\r\n", n, m);
        else
            rt_snprintf(buf, sizeof(buf), "Saved S%d p%d = %d deg, %dms\r\n", n, m, deg, ms);
        shell_write(buf);
        return;
    }

    if (argv[1][0] == 'M' && (argv[1][1] == 'P' || argv[1][1] == 'p') && argv[1][2] == '\0') {
        if (argv[2][0] != 'P' && argv[2][0] != 'p') {
            shell_write("Usage: set MP P<m> <UP|DN|OFF>\r\n"); return;
        }
        int m;
        if (!_atoi_strict(argv[2] + 1, &m) || m < 1 || m > PRESET_POSITIONS) { shell_write("Preset must be p1..p8\r\n"); return; }

        uint8_t st;
        if      (_ci_eq(argv[3], "UP"))  st = PRESET_PUSHROD_UP;
        else if (_ci_eq(argv[3], "DN"))  st = PRESET_PUSHROD_DN;
        else if (_ci_eq(argv[3], "OFF")) st = PRESET_PUSHROD_OFF;
        else { shell_write("State must be UP|DN|OFF\r\n"); return; }

        preset_set_pushrod((uint8_t)m, st);
        char buf[48];
        rt_snprintf(buf, sizeof(buf), "Saved MP p%d = %s\r\n", m,
                    st == PRESET_PUSHROD_UP ? "UP" : (st == PRESET_PUSHROD_DN ? "DN" : "OFF"));
        shell_write(buf);
        return;
    }

    if (argv[1][0] == 'C' && (argv[1][1] == 'P' || argv[1][1] == 'p') && argv[1][2] == '\0') {
        if (argv[2][0] != 'P' && argv[2][0] != 'p') {
            shell_write("Usage: set CP P<m> <UP|DN|OFF>\r\n"); return;
        }
        int m;
        if (!_atoi_strict(argv[2] + 1, &m) || m < 1 || m > PRESET_POSITIONS) { shell_write("Preset must be p1..p8\r\n"); return; }

        uint8_t st;
        if      (_ci_eq(argv[3], "UP"))  st = PRESET_PUSHROD_UP;
        else if (_ci_eq(argv[3], "DN"))  st = PRESET_PUSHROD_DN;
        else if (_ci_eq(argv[3], "OFF")) st = PRESET_PUSHROD_OFF;
        else { shell_write("State must be UP|DN|OFF\r\n"); return; }

        preset_set_cprod((uint8_t)m, st);
        char buf[48];
        rt_snprintf(buf, sizeof(buf), "Saved CP p%d = %s\r\n", m,
                    st == PRESET_PUSHROD_UP ? "UP" : (st == PRESET_PUSHROD_DN ? "DN" : "OFF"));
        shell_write(buf);
        return;
    }

    shell_write("Usage: set S<n> P<m> <deg|OFF> [ms]  |  set MP P<m> <UP|DN|OFF>  |  set CP P<m> <UP|DN|OFF>\r\n");
}

/* show S<n> | show MP | show <thread_name> —— 只读查看，不改变任何状态 */
static void cmd_show(int argc, char **argv)
{
    if (argc < 2) {
        shell_write("Usage: show S<n> | show MP | show CP | show <thread_name>\r\n");
        return;
    }

    /* show S<n> : 舵机 n 的全部预设 */
    if ((argv[1][0] == 'S' || argv[1][0] == 's') && argv[1][1] != '\0') {
        int n;
        if (!_atoi_strict(argv[1] + 1, &n) || n < 0 || n > 15) {
            shell_write("Servo must be S0..S15\r\n");
            return;
        }
        char buf[64];
        rt_snprintf(buf, sizeof(buf), "Servo %d presets:\r\n", n);
        shell_write(buf);
        for (int m = 1; m <= PRESET_POSITIONS; m++) {
            uint16_t deg, ms;
            preset_get_servo((uint8_t)n, (uint8_t)m, &deg, &ms);
            if (deg == PRESET_SERVO_OFF)
                rt_snprintf(buf, sizeof(buf), "  p%d = OFF\r\n", m);
            else
                rt_snprintf(buf, sizeof(buf), "  p%d = %d deg, %d ms\r\n", m, deg, ms);
            shell_write(buf);
        }
        return;
    }

    /* show MP : 推杆全部预设 */
    if (_ci_eq(argv[1], "MP")) {
        shell_write("Pushrod presets:\r\n");
        char buf[48];
        for (int m = 1; m <= PRESET_POSITIONS; m++) {
            uint8_t st;
            preset_get_pushrod((uint8_t)m, &st);
            const char *s = (st == PRESET_PUSHROD_UP) ? "UP"
                          : (st == PRESET_PUSHROD_DN) ? "DN" : "OFF";
            rt_snprintf(buf, sizeof(buf), "  p%d = %s\r\n", m, s);
            shell_write(buf);
        }
        return;
    }

    /* show CP : CP 推杆全部预设 */
    if (_ci_eq(argv[1], "CP")) {
        shell_write("CP rod presets:\r\n");
        char buf[48];
        for (int m = 1; m <= PRESET_POSITIONS; m++) {
            uint8_t st;
            preset_get_cprod((uint8_t)m, &st);
            const char *s = (st == PRESET_PUSHROD_UP) ? "UP"
                          : (st == PRESET_PUSHROD_DN) ? "DN" : "OFF";
            rt_snprintf(buf, sizeof(buf), "  p%d = %s\r\n", m, s);
            shell_write(buf);
        }
        return;
    }

    /* show <thread_name> : 线程的全部内部指令 */
    int idx = thread_store_index(argv[1]);
    if (idx >= 0) {
        char buf[64];
        rt_snprintf(buf, sizeof(buf), "Thread %s:\r\n", thread_store_name(idx));
        shell_write(buf);
        const char *text = thread_store_text(idx);
        if (!*text) {
            shell_write("  (empty)\r\n");
        } else {
            int ln = 1;
            const char *ls = text;
            while (*ls) {
                const char *nl = ls;
                while (*nl && *nl != '\n') nl++;
                int len = (int)(nl - ls);
                if (len > LINE_MAX - 1) len = LINE_MAX - 1;
                char line[LINE_MAX + 8];
                int off = rt_snprintf(line, sizeof(line), "%2d  ", ln++);
                rt_memcpy(line + off, ls, len);
                line[off + len] = '\0';
                shell_write(line);
                shell_write("\r\n");
                ls = (*nl == '\n') ? nl + 1 : nl;
            }
        }
        return;
    }

    shell_write("Unknown: use show S<n> | show MP | show CP | show <thread_name>\r\n");
}

static void cmd_stop(int argc, char **argv)
{
    thread_store_stop();
    servo_motion_stop_all();
    pushrod_off();   /* MP: relays 6/7/8 off */
    cprod_off();     /* CP: RL4 on, RL5 off */
    lift_off();      /* LIFT: RL2/RL3 off */
    relay_set(1, 0); /* free relay 1 off */
    shell_write("Stopped, actuators reset\r\n");
}

static void cmd_hardware(int argc, char **argv)
{
    shell_write("\r\n");
    shell_write("  === Hardware Control ===\r\n");
    shell_write("  S<n> <deg> [ms] - servo n (0-15): rotate to deg; optional ms (0..10000)\r\n");
    shell_write("  S<n> OFF        - servo n off (no PWM)\r\n");
    shell_write("  S<n> P<m>       - servo n apply preset m (1-8)\r\n");
    shell_write("  RL<n> ON|OFF    - relay n (1 only); RL2-3 = LIFT, RL4-5 = CP, RL6-8 = MP\r\n");
    shell_write("  MP <UP|DN> [ms] - push rod: up/down, 0..20000 ms (omit = continuous)\r\n");
    shell_write("  MP OFF/STOP/P<m> - push rod off / stop / preset\r\n");
    shell_write("  MP END           - push rod DN until limit switch (PC0) triggers\r\n");
    shell_write("  CP <UP|DN> [ms] - CP push rod: up/down, 0..20000 ms (omit = continuous)\r\n");
    shell_write("  CP OFF/STOP/P<m> - CP push rod off / stop / preset\r\n");
    shell_write("  LIFT <UP|DN> [ms|end] - lift: up/down, ms timed (stops early on limit)\r\n");
    shell_write("  LIFT OFF/STOP    - lift off / stop (both relays off)\r\n");
    shell_write("  LIFT DN end      - descend until bottom limit (S2/PC1) triggers\r\n");
    shell_write("  set S<n> P<m> <deg|OFF> [ms] - save servo preset m\r\n");
    shell_write("  set MP P<m> <UP|DN|OFF>      - save pushrod preset m\r\n");
    shell_write("  set CP P<m> <UP|DN|OFF>      - save CP pushrod preset m\r\n");
    shell_write("  Run <name>      - run thread script (Thread1..16)\r\n");
    shell_write("  edit <name>     - edit thread; then insert/edit/delete/quit\r\n");
    shell_write("  stop            - stop script + reset all actuators\r\n");
    shell_write("\r\n");
}

static const struct shell_cmd _table[] = {
    {"help",  "show help",       cmd_help},
    {"led",   "led <on|off|blink>", cmd_led},
    {"info",  "system info",     cmd_info},
    {"reset", "reset mcu",       cmd_reset},
    {"ota",   "reboot to OTA mode", cmd_ota},
    {"who",   "connections",     cmd_who},
    {"oled",  "oled <line> <text>", cmd_oled},
    {"wifi",  "wifi save/show/delete", cmd_wifi},
    {"hardware", "servo/relay control help", cmd_hardware},
};
#define N_CMD (sizeof(_table)/sizeof(_table[0]))

/* -------- parser -------- */

/* hardware commands are blocked while a thread script is running */
static int _is_hw_cmd(const char *c)
{
    if (!c[0]) return 0;
    char c0 = c[0];
    if (c0 >= 'a' && c0 <= 'z') c0 -= 32;
    if (c0 == 'S' || c0 == 'M' || c0 == 'R' || c0 == 'C') return 1;
    if (_ci_eq(c, "lift")) return 1;
    if (_ci_eq(c, "set"))  return 1;
    if (_ci_eq(c, "run"))  return 1;
    if (_ci_eq(c, "edit")) return 1;
    return 0;
}

static void shell_exec(char *line)
{
    char *argv[8]; int argc=0; char *p=line;
    while (*p==' ') p++;
    while (*p && argc<8) {
        argv[argc++]=p;
        while (*p && *p!=' ') p++;
        if (*p) { *p=0; p++; while (*p==' ') p++; }
    }
    if (!argc) return;

    /* ---- edit mode: only insert / edit / quit ---- */
    if (_in_edit) {
        if (_ci_eq(argv[0], "quit")) {
            thread_store_edit_quit();
            _in_edit = 0;
            shell_write("Thread saved\r\n");
        } else if (_ci_eq(argv[0], "insert")) {
            if (argc < 3) { shell_write("Usage: insert <after_line> <count>\r\n"); return; }
            int after = atoi(argv[1]);
            int count = atoi(argv[2]);
            if (thread_store_edit_insert(after, count) != 0)
                shell_write("insert failed (line limit or bad args)\r\n");
            else
                thread_store_edit_show(shell_write);
        } else if (_ci_eq(argv[0], "delete") || _ci_eq(argv[0], "del")) {
            if (argc < 2) { shell_write("Usage: delete <line> [count]\r\n"); return; }
            int ln = atoi(argv[1]);
            int count = (argc >= 3) ? atoi(argv[2]) : 1;
            if (thread_store_edit_delete(ln, count) != 0)
                shell_write("delete failed (bad line/count)\r\n");
            else
                thread_store_edit_show(shell_write);
        } else if (_ci_eq(argv[0], "edit")) {
            if (argc < 3) { shell_write("Usage: edit <line> <content>\r\n"); return; }
            int ln = atoi(argv[1]);
            /* 把 argv[2..argc-1] 重新拼回完整内容（shell 已按空格切分，
             * 不能只取 argv[2]，否则 "edit 1 mp dn 1000" 会只剩 "mp"） */
            char content[LINE_MAX];
            int off = 0;
            for (int i = 2; i < argc; i++) {
                if (off >= LINE_MAX - 1) break;
                if (i > 2) content[off++] = ' ';
                const char *s = argv[i];
                while (*s && off < LINE_MAX - 1) content[off++] = *s++;
            }
            content[off] = '\0';
            int rc = thread_store_edit_set(ln, content);
            if (rc == -1)
                shell_write("Invalid line number\r\n");
            else if (rc == -2)
                shell_write("Invalid command (S<n>/RL<n>/MP/CP/D/Run)\r\n");
            else
                thread_store_edit_show(shell_write);
        } else {
            shell_write("Editing: only insert / edit / delete / quit accepted\r\n");
        }
        return;
    }

    /* ---- stop: always allowed ---- */
    if (_ci_eq(argv[0], "stop")) { cmd_stop(argc, argv); return; }

    /* ---- show: read-only, always allowed ---- */
    if (_ci_eq(argv[0], "show")) { cmd_show(argc, argv); return; }

    /* ---- running script: block hardware commands ---- */
    if (thread_store_running() && _is_hw_cmd(argv[0])) {
        shell_write("Thread is running\r\n");
        return;
    }

    /* ---- edit / Run / set ---- */
    if (_ci_eq(argv[0], "edit")) {
        if (argc < 2) { shell_write("Usage: edit <thread_name>\r\n"); return; }
        int idx = thread_store_index(argv[1]);
        if (idx < 0) { shell_write("Thread not found\r\n"); return; }
        thread_store_edit_begin(idx);
        _in_edit = 1;
        thread_store_edit_show(shell_write);
        return;
    }
    if (_ci_eq(argv[0], "run")) {
        if (argc < 2) { shell_write("Usage: Run <thread_name>\r\n"); return; }
        int idx = thread_store_index(argv[1]);
        if (idx < 0) { shell_write("Thread not found\r\n"); return; }
        if (thread_store_run(idx) != 0)
            shell_write("Thread is running\r\n");
        else
            shell_write("Thread started\r\n");
        return;
    }
    if (_ci_eq(argv[0], "set")) { cmd_set(argc, argv); return; }

    /* dynamic hardware commands: S<n> = servo (0..15), RL<n> = relay (1..8) */
    if ((argv[0][0] == 'S' || argv[0][0] == 's') && argv[0][1] != '\0') {
        int n;
        if (_atoi_strict(argv[0] + 1, &n) && n >= 0 && n <= 15) { cmd_servo(argc, argv, n); return; }
    }
    if ((argv[0][0] == 'R' || argv[0][0] == 'r') &&
        (argv[0][1] == 'L' || argv[0][1] == 'l') && argv[0][2] != '\0') {
        int n;
        if (_atoi_strict(argv[0] + 2, &n) && n >= 1 && n <= 8) { cmd_relay(argc, argv, n); return; }
    }
    if ((argv[0][0] == 'M' || argv[0][0] == 'm') &&
        (argv[0][1] == 'P' || argv[0][1] == 'p') && argv[0][2] == '\0') {
        cmd_pushrod(argc, argv); return;
    }
    if ((argv[0][0] == 'C' || argv[0][0] == 'c') &&
        (argv[0][1] == 'P' || argv[0][1] == 'p') && argv[0][2] == '\0') {
        cmd_cprod(argc, argv); return;
    }
    if (_ci_eq(argv[0], "LIFT")) {
        cmd_lift(argc, argv); return;
    }

    for (int i=0; i<N_CMD; i++)
        if (rt_strcmp(argv[0], _table[i].name)==0)
            { _table[i].func(argc,argv); return; }
    shell_write("?: "); shell_write(argv[0]); shell_write("\r\n");
}

/* -------- UART helpers -------- */
static rt_device_t _dev_local  = RT_NULL;
static rt_device_t _dev_remote = RT_NULL;

static rt_device_t open_dev(const char *name, int use_int_rx)
{
    rt_device_t dev = rt_device_find(name);
    if (!dev) { rt_kprintf("[SHELL] %s not found\r\n", name); return RT_NULL; }
    if (dev->ref_count > 0) return dev;
    int flags = RT_DEVICE_OFLAG_RDWR;
    if (use_int_rx) flags |= RT_DEVICE_FLAG_INT_RX;
    if (rt_device_open(dev, flags)) {
        rt_kprintf("[SHELL] %s open fail\r\n", name); return RT_NULL;
    }
    return dev;
}

static int proc_char(char ch, rt_device_t src, char *line, int *pos, int max)
{
    if (ch==0x08 || ch==0x7F) {
        if (*pos>0) { (*pos)--; rt_device_write(src,0,"\b \b",3); }
        return 0;
    }
    if (ch=='\r' || ch=='\n') {
        rt_device_write(src,0,"\r\n",2);
        return (*pos>0) ? 1 : 0;
    }
    if ((unsigned char)ch==0xFF) {
        char d[2]; rt_device_read(src,0,&d[0],1); rt_device_read(src,0,&d[1],1);
        return 0;
    }
    if (*pos<max-1 && ch>=0x20 && ch<=0x7E) {
        line[(*pos)++]=ch; rt_device_write(src,0,&ch,1);
    }
    return 0;
}

/* ===================================================================
 *  shell_reader_thread  (was shell_thread — polls UART1+UART2)
 *  =================================================================== */
static void shell_reader_thread(void *p)
{
    char line[SHELL_LINE_MAX]; int pos=0; char ch;

    rt_thread_mdelay(600);

    _dev_local = open_dev("uart1", 0);
    if (!_dev_local) return;

    /* Try to open UART2. If ESP8266 init succeeded, ref_count>0
     * and we just get the existing handle. Otherwise harmless. */
    _dev_remote = open_dev("uart2", 1);
    if (_dev_remote) _remote_dev = _dev_remote;

    rt_kprintf("\033[34m\r\n");
    rt_kprintf("+======================================================+\r\n");
    rt_kprintf("|    ___    ___    ___  _______ _________  ________    |\r\n");
    rt_kprintf("|    |  |   |  |   |  |/  _____|  ____   |/   _____|   |\r\n");
    rt_kprintf("|    |  |   |  |   |  |  |     |  |   |  |   |_____    |\r\n");
    rt_kprintf("|    |  |   |  |   |  |  |     |  |___|  |\\_____   \\   |\r\n");
    rt_kprintf("|    |  |___|  |___|  |  |_____|  ____   |_____ |  |   |\r\n");
    rt_kprintf("|    |______|\\________/\\_______|__|   |__|\\________/   | \r\n");
    rt_kprintf("|                                                      |\r\n");
    rt_kprintf("+======================================================+\r\n");
    rt_kprintf("\033[0m");
    rt_kprintf("type 'help' for commands\r\n");
    if (_dev_remote) rt_kprintf("[SHELL] local UART1 + remote UART2 (telnet)\r\n");
    rt_kprintf(SHELL_PROMPT);

    while (1) {
        /* delayed banner: send 500ms after connect, non-blocking */
        if (_banner_link >= 0 && rt_tick_get() - _banner_tick > rt_tick_from_millisecond(500)) {
            int save = _active_link, bl = _banner_link;
            _banner_link = -1;
            _active_link = bl;
            shell_write("\033[34m\r\n");
            shell_write("+======================================================+\r\n");
            shell_write("|    ___    ___    ___  _______ _________  ________    |\r\n");
            shell_write("|    |  |   |  |   |  |/  _____|  ____   |/   _____|   |\r\n");
            shell_write("|    |  |   |  |   |  |  |     |  |   |  |   |_____    |\r\n");
            shell_write("|    |  |   |  |   |  |  |     |  |___|  |\\_____   \\   |\r\n");
            shell_write("|    |  |___|  |___|  |  |_____|  ____   |_____ |  |   |\r\n");
            shell_write("|    |______|\\________/\\_______|__|   |__|\\________/   | \r\n");
            shell_write("|                                                      |\r\n");
            shell_write("+======================================================+\r\n");
            shell_write("\033[0m");
            shell_write("type 'help' for commands\r\n");
            remote_flush();
            _active_link = save;
        }

        /* poll local uart1 */
        while (rt_device_read(_dev_local, 0, &ch, 1) == 1) {
            if (proc_char(ch, _dev_local, line, &pos, SHELL_LINE_MAX)) {
                _active_link = -1; line[pos]=0; shell_exec(line); pos=0;
                remote_flush();
            }
        }

        /* handle remote uart2 (ESP8266 CIPSERVER +IPD protocol) */
        if (_dev_remote) {
            /* ---- take mutex: prevent writer from stealing IPD bytes ---- */
            rt_mutex_take(&_remote_mutex, RT_WAITING_FOREVER);

            char rbuf[640];
            int rlen = 0;

            /* prepend any data saved by writer's remote_send */
            if (_save_len > 0)
            {
                int n = _save_len < (int)sizeof(rbuf)-1 ? _save_len : (int)sizeof(rbuf)-1;
                rt_memcpy(rbuf, _save_buf, n);
                rlen = n;
                _save_len = 0;
            }

            /* read new data */
            while (rt_device_read(_dev_remote, 0, &ch, 1) == 1) {
                rbuf[rlen++] = ch;
                if (rlen >= (int)sizeof(rbuf) - 1) break;
            }
            rbuf[rlen] = '\0';

            /* ---- parse ALL +IPD packets inside critical section ----
             * Scan rbuf for every +IPD,0,N: and concatenate data bytes
             * into tmp[].  Multiple +IPDs in one read batch → all captured.
             */
            char tmp[256]; int tlen = 0; int has_ipd = 0;
            int scan = 0;

            while (scan < rlen - 8)
            {
                /* find next +IPD,<link>, */
                if (!(rbuf[scan] == '+' && rbuf[scan+1] == 'I' && rbuf[scan+2] == 'P' &&
                      rbuf[scan+3] == 'D' && rbuf[scan+4] == ','))
                    { scan++; continue; }
                /* parse link number */
                int link = 0;
                char *p = rbuf + scan + 5;
                if (*p < '0' || *p > '4' || *(p+1) != ',') { scan++; continue; }
                link = *p - '0'; p += 2;  /* skip "N," */
                /* parse datalen */
                int datalen = 0;
                while (*p >= '0' && *p <= '9') { datalen = datalen * 10 + (*p - '0'); p++; }
                if (!(*p == ':' && datalen > 0 && datalen < 256))
                    { scan++; continue; }

                has_ipd = 1; _active_link = link;
                p++;  /* skip ':' */
                int header_end = (int)(p - rbuf);
                int already = rlen - header_end;
                if (already > datalen) already = datalen;

                /* copy bytes already in rbuf */
                for (int i = 0; i < already && tlen < 255; i++)
                    tmp[tlen++] = p[i];

                /* read remaining payload bytes */
                int remaining = datalen - already;
                if (remaining > 0) {
                    int deadline = rt_tick_get() + rt_tick_from_millisecond(100);
                    while (remaining > 0 && tlen < 255) {
                        char dc;
                        if (rt_device_read(_dev_remote, 0, &dc, 1) == 1)
                            { tmp[tlen++] = dc; remaining--; }
                        else if (rt_tick_get() > deadline)
                            break;
                    }
                }

                /* advance scan past this +IPD (p points to start of data) */
                scan = header_end + datalen;
            }

            /* save tail (unprocessed bytes) for next cycle — prevents
             * data loss when +IPD header is split across polling cycles */
            if (scan < rlen) {
                int tail = rlen - scan;
                if (tail > (int)sizeof(_save_buf) - 1)
                    tail = (int)sizeof(_save_buf) - 1;
                rt_memcpy(_save_buf, rbuf + scan, tail);
                _save_len = tail;
            }

            /* release UART2 — writer can now do CIPSEND */
            rt_mutex_release(&_remote_mutex);

            /* ---- connection events ---- */
            int save_link = _active_link;  /* save IPD link, restore after events */
            {
                char *cp = rt_strstr(rbuf, ",CONNECT");
                if (cp && cp > rbuf && *(cp-1) >= '0' && *(cp-1) <= '4')
                {
                    int clink = *(cp-1) - '0';
                    rt_kprintf("[telnet] L%d connected\r\n", clink);
                    OLED_ShowString(1, 16, "R");
                    _banner_link = clink; _banner_tick = rt_tick_get();
            }
            {
                char *dp = rt_strstr(rbuf, ",CLOSED");
                if (dp && dp > rbuf && *(dp-1) >= '0' && *(dp-1) <= '4')
                {
                    int dlink = *(dp-1) - '0';
                    rt_kprintf("[telnet] L%d disconnected\r\n", dlink);
                    _remote_len[dlink] = 0;
                    OLED_ShowChar(1, 16, ' ');
                }
            }
            if (!_wifi_connecting && rt_strstr(rbuf, "WIFI DISCONNECT"))
            {
                rt_kprintf("[WiFi] DISCONNECTED\r\n");
                OLED_ShowString(1, 1, "WiFi Disconnect ");
            }

            _active_link = save_link;  /* restore IPD link */
            /* ---- process IPD data ---- */
            if (has_ipd)
            {
                for (int i = 0; i < tlen; i++)
                {
                    unsigned char c = (unsigned char)tmp[i];

                    /* IAC telnet negotiation: respond + skip */
                    if (c == 0xFF && i + 2 < tlen)
                    {
                        unsigned char cmd = (unsigned char)tmp[i+1];
                        unsigned char opt = (unsigned char)tmp[i+2];
                        if (cmd == 0xFB || cmd == 0xFD) {
                            char r[3];
                            r[0] = (char)0xFF;
                            if (opt == 0x03) {
                                r[1] = (cmd == 0xFB) ? (char)0xFD : (char)0xFB;
                            } else if (opt == 0x01) {
                                r[1] = (cmd == 0xFB) ? (char)0xFD : (char)0xFC;
                            } else {
                                r[1] = (cmd == 0xFB) ? (char)0xFE : (char)0xFC;
                            }
                            r[2] = (char)opt;
                            while (_tx_busy) rt_thread_mdelay(5);
                            writer_post(1, _active_link, r, 3);
                        }
                        i += 2;
                        continue;
                    }

                    /* append to remote buffer for active link */
                    if (c == '\r' || c == '\n')
                    {
                        int link = _active_link;
                        if (link < 0 || link > 4) { remote_flush(); continue; }
                        _out_len[link] = 2;
                        _out_buf[link][0] = '\r'; _out_buf[link][1] = '\n';
                        _out_buf[link][2] = '\0';
                        if (_remote_len[link] > 0)
                        {
                            _remote_buf[link][_remote_len[link]] = '\0';
                            rt_kprintf("[CMD: '%s']\r\n", _remote_buf[link]);
                            shell_exec(_remote_buf[link]);
                            _remote_len[link] = 0;
                        }
                        remote_flush();
                    }
                    else if (c == '\b' || c == 0x7F)
                    {
                        int link = _active_link;
                        if (link >= 0 && link < 5 && _remote_len[link] > 0)
                            _remote_len[link]--;
                    }
                    else if (c >= 0x20)
                    {
                        int link = _active_link;
                        if (link >= 0 && link < 5 && _remote_len[link] < 255)
                            _remote_buf[link][_remote_len[link]++] = (char)c;
                    }
                }
            }
            }  /* close connection-events block */
        }

        rt_thread_mdelay(10);
    }
}

/* ===================================================================
 *  shell_writer_thread  —  drains _tx_sem, does CIPSEND (may block)
 *  =================================================================== */
static void shell_writer_thread(void *p)
{
    while (1) {
        rt_sem_take(&_tx_sem, RT_WAITING_FOREVER);

        rt_mutex_take(&_remote_mutex, RT_WAITING_FOREVER);
        if (_tx_raw)
            remote_send_raw(_tx_link, _tx_buf, _tx_len);
        else
            remote_send(_tx_link, _tx_buf);
        rt_mutex_release(&_remote_mutex);
        _tx_busy = 0;
    }
}

/* ===================================================================
 *  shell_init()
 *  =================================================================== */
int shell_init(void)
{
    /* mutex for UART2 shared access */
    rt_mutex_init(&_remote_mutex, "rmtx", RT_IPC_FLAG_FIFO);

    /* mutex for the per-link output buffer (shell_write: reader + runner) */
    rt_mutex_init(&_out_mutex, "outtx", RT_IPC_FLAG_FIFO);

    /* semaphore: reader posts when output is ready */
    rt_sem_init(&_tx_sem, "txsem", 0, RT_IPC_FLAG_FIFO);

    /* writer thread (higher prio → prompt CIPSEND drain) */
    rt_thread_t tw = rt_thread_create("shWr", shell_writer_thread, RT_NULL,
                                       SHELL_WRITER_STACK_SIZE,
                                       SHELL_WRITER_PRIORITY, 10);
    if (tw) rt_thread_startup(tw);

    /* reader thread */
    rt_thread_t tr = rt_thread_create("shRd", shell_reader_thread, RT_NULL,
                                       SHELL_READER_STACK_SIZE,
                                       SHELL_READER_PRIORITY, 10);
    if (!tr) return -1;
    rt_thread_startup(tr);

    return RT_EOK;
}
