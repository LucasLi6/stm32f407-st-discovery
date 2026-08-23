/*
 * ESP8266 WiFi module driver (AT command mode)
 *
 * Hardware: ESP-01S -> STM32 USART2
 *   PA2 = USART2_TX -> ESP RX
 *   PA3 = USART2_RX -> ESP TX
 *
 * Configures ESP8266 as TCP Server on port 23 (telnet).
 */

#include "esp8266.h"
#include "../oled/OLED.h"
#include "../wifi_config.h"

#define DBG_TAG  "esp8266"
#define DBG_LVL  DBG_LOG
#include <rtdbg.h>

/* ---------- ESP8266 serial device ---------- */
#define ESP8266_DEVICE  "uart2"

static rt_device_t _esp_dev = RT_NULL;
char g_esp8266_ip[17] = {0};  /* exposed for OLED re-display */
char g_connected_ssid[33] = {0};  /* SSID that actually connected */

/* ---------- AT command helper ---------- */

/*
 * Read all available data from ESP8266 into buf.
 * Returns number of bytes read.
 */
static int esp_read(char *buf, int max, int timeout_ms)
{
    int total = 0;
    int tick = rt_tick_get();

    while (total < max - 1)
    {
        char ch;
        if (rt_device_read(_esp_dev, 0, &ch, 1) == 1)
        {
            buf[total++] = ch;
            tick = rt_tick_get();
            continue;
        }

        /* no data in ring buffer — check timeout */
        if (rt_tick_get() - tick > rt_tick_from_millisecond(timeout_ms))
            break;

        /* yield CPU — ring buffer is 256B, ~115B/10ms, safe */
        rt_thread_mdelay(5);
    }
    buf[total] = '\0';
    return total;
}

/*
 * Send AT command and wait for expected response.
 * Returns 0 on success, -1 on timeout or mismatch.
 */
static int at_send(const char *cmd, const char *expect, int timeout_ms)
{
    char buf[256];
    int  len;

    /* drain any stale data (with timeout — don't loop forever) */
    {
        int tick = rt_tick_get();
        while (rt_device_read(_esp_dev, 0, buf, 1) > 0) {
            if (rt_tick_get() - tick > rt_tick_from_millisecond(200))
                break;
        }
    }

    /* send command */
    rt_device_write(_esp_dev, 0, cmd, rt_strlen(cmd));

    /* wait for response, collecting all characters */
    len = esp_read(buf, sizeof(buf), timeout_ms);

    LOG_D("cmd: %s", cmd);
    LOG_D("rsp(%d): %s", len, buf);

    if (len > 0 && rt_strstr(buf, expect) != RT_NULL)
        return RT_EOK;
    return -RT_ERROR;
}

/*
 * Send command, retry up to `retries` times.
 */
static int at_send_retry(const char *cmd, const char *expect,
                         int timeout_ms, int retries)
{
    for (int i = 0; i < retries; i++)
    {
        if (at_send(cmd, expect, timeout_ms) == RT_EOK)
            return RT_EOK;
        rt_thread_mdelay(500);
    }
    return -RT_ERROR;
}

/*
 * Try AT handshake at a specific baud rate.
 * Returns 0 on success.
 */
static int try_baud(int baud)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;
    cfg.baud_rate = baud;

    rt_device_control(_esp_dev, RT_DEVICE_CTRL_CONFIG, &cfg);
    rt_thread_mdelay(100);

    LOG_I("Trying %d baud...", baud);
    return at_send_retry("AT\r\n", "OK", 500, 3);
}

/* ---------- public API ---------- */

int esp8266_init(void)
{
    LOG_I("Initializing ESP8266 on %s ...", ESP8266_DEVICE);

    /* 1. open uart2 in polling mode (don't use INT for init phase) */
    _esp_dev = rt_device_find(ESP8266_DEVICE);
    if (_esp_dev == RT_NULL)
    {
        LOG_E("device %s not found!", ESP8266_DEVICE);
        OLED_ShowString(1, 1, "ESP8266 Error   ");
        return -RT_ERROR;
    }

    if (_esp_dev->ref_count > 0)
        rt_device_close(_esp_dev);

    if (rt_device_open(_esp_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("open %s failed!", ESP8266_DEVICE);
        OLED_ShowString(1, 1, "ESP8266 Error   ");
        return -RT_ERROR;
    }

    /* 2. ensure ESP is in AT mode (exit transparent if stuck) */
    rt_thread_mdelay(1000);
    rt_device_write(_esp_dev, 0, "+++", 3);
    rt_thread_mdelay(1500);
    { char d[64]; while(rt_device_read(_esp_dev,0,d,sizeof(d))>0); }  /* drain */

    /* 3. try 115200 baud */
    if (try_baud(115200) != RT_EOK)
    {
        LOG_E("ESP8266 not responding at 115200 baud");
        LOG_E("Check: wiring, power (3.3V >500mA), GPIO0=HIGH, CH_PD=HIGH");
        OLED_ShowString(1, 1, "ESP8266 Error   ");
        rt_device_close(_esp_dev);
        return -RT_ERROR;
    }
    LOG_I("ESP8266 handshake OK at 115200 baud");

    /* 4. station mode */
    at_send("AT+CWMODE=1\r\n", "OK", 1000);

    /* 5. connect to WiFi using stored credentials (priority order) */
    if (wifi_config_connect(_esp_dev) != 0)
    {
        LOG_E("WiFi connect failed (no saved networks or all failed).");
        OLED_ShowString(1, 1, "No WiFi saved  ");
        rt_device_close(_esp_dev);
        return -RT_ERROR;
    }
    LOG_I("WiFi connected");

    /* 6. multi-connection mode */
    at_send("AT+CIPMUX=1\r\n", "OK", 1000);

    /* 7. start TCP server on port 23 */
    if (at_send("AT+CIPSERVER=1,23\r\n", "OK", 1000) != RT_EOK)
    {
        LOG_E("TCP Server start failed");
        OLED_ShowString(1, 1, "ESP8266 Error   ");
        rt_device_close(_esp_dev);
        return -RT_ERROR;
    }
    LOG_I("TCP Server listening on port 23");

    /* 8. query and print IP */
    {
        char buf[256], *start, *end;
        int len;

        {
            int tick = rt_tick_get();
            while (rt_device_read(_esp_dev, 0, buf, 1) > 0) {
                if (rt_tick_get() - tick > rt_tick_from_millisecond(200))
                    break;
            }
        }
        rt_device_write(_esp_dev, 0, "AT+CIFSR\r\n", 10);
        len = esp_read(buf, sizeof(buf), 1000);

        if (len > 0)
        {
            start = rt_strstr(buf, "+CIFSR:STAIP,\"");
            if (start)
            {
                start += 14;
                end = rt_strstr(start, "\"");
                if (end) *end = '\0';

                /* show IP on OLED line 1 */
                {
                    rt_snprintf(g_esp8266_ip, sizeof(g_esp8266_ip), "%-16s", start);
                    OLED_ShowString(1, 1, g_esp8266_ip);
                }

                rt_kprintf("\r\n");
                rt_kprintf("  +----------------------------------+\r\n");
                rt_kprintf("  |  ESP8266 WiFi Connected          |\r\n");
                rt_kprintf("  |  SSID: %-24s |\r\n",
                         g_connected_ssid[0] ? g_connected_ssid : "(saved)");
                rt_kprintf("  |  IP:   %-24s  |\r\n", start);
                rt_kprintf("  |  Port: 23 (telnet)               |\r\n");
                rt_kprintf("  +----------------------------------+\r\n");
                rt_kprintf("  |  telnet %-23s  |\r\n", start);
                rt_kprintf("  +----------------------------------+\r\n\r\n");
            }
            else
            {
                rt_kprintf("[ESP8266] IP query raw:\r\n%s\r\n", buf);
            }
        }
    }

    /* 9. keep UART2 open — shell picks it up via ref_count */
    return RT_EOK;
}

rt_device_t esp8266_get_device(void)
{
    return _esp_dev;
}
