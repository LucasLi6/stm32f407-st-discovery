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

static void shell_write(const char *str)
{
    rt_kprintf("%s", str);  /* serial console sees everything */
    int link = _active_link;
    if (link >= 0 && link < 5) {
        int slen = rt_strlen(str);
        if (_out_len[link] + slen < 768 - 1) {
            rt_memcpy(_out_buf[link] + _out_len[link], str, slen);
            _out_len[link] += slen;
            _out_buf[link][_out_len[link]] = '\0';
        }
    }
}

/* broadcast to all connected remote terminals */
static void shell_write_all(const char *str)
{
    rt_kprintf("%s", str);
    int slen = rt_strlen(str);
    for (int l = 0; l < 5; l++) {
        if (_out_len[l] + slen < 768 - 1) {
            rt_memcpy(_out_buf[l] + _out_len[l], str, slen);
            _out_len[l] += slen;
            _out_buf[l][_out_len[l]] = '\0';
        }
    }
}

/* flush all link buffers (call after broadcast) */
static void remote_flush_all(void)
{
    for (int l = 0; l < 5; l++) {
        if (_remote_dev && _out_len[l] > 0) {
            writer_post(0, l, _out_buf[l], _out_len[l] + 1);
            _out_len[l] = 0;
            while (_tx_busy) rt_thread_mdelay(10);  /* wait for writer to finish */
        }
    }
    rt_kprintf(SHELL_PROMPT);
}

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
    if (_out_len[link] + plen < 768 - 1) {
        rt_memcpy(_out_buf[link] + _out_len[link], SHELL_PROMPT, plen);
        _out_len[link] += plen;
        _out_buf[link][_out_len[link]] = '\0';
    }
    while (_tx_busy) rt_thread_mdelay(5);  /* wait for previous send to finish */
    if (_remote_dev && _out_len[link] > 0)
        writer_post(0, link, _out_buf[link], _out_len[link] + 1);
    _out_len[link] = 0;
    rt_kprintf("%s", SHELL_PROMPT);
}

/* -------- commands -------- */
static void cmd_help(int argc, char **argv)
{
    shell_write("\r\n");
    shell_write("  === STM32F103 Shell ===\r\n");
    shell_write("  help          - show help\r\n");
    shell_write("  led  on/off   - turn LED on/off\r\n");
    shell_write("  led  blink    - SOS blink\r\n");
    shell_write("  info          - system info\r\n");
    shell_write("  reset         - reset MCU\r\n");
    shell_write("  ota [serial]  - OTA via serial (Ymodem)\r\n");
    shell_write("  ota wifi      - OTA via WiFi (TCP)\r\n");
    shell_write("  who           - show connections\r\n");
    shell_write("  oled <L> <T>  - show text on OLED line 2-4\r\n");
    shell_write("  oled clear    - clear OLED screen\r\n");
    shell_write("  wifi show     - list saved WiFi networks\r\n");
    shell_write("  wifi save     - save WiFi <ssid> <pass> [prio]\r\n");
    shell_write("  wifi delete   - delete WiFi by priority\r\n");
    shell_write("  wifi exc      - swap two WiFi priorities\r\n");
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
    else {
        shell_write("Unknown wifi sub-command.\r\n");
    }
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
};
#define N_CMD (sizeof(_table)/sizeof(_table[0]))

/* -------- parser -------- */
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
