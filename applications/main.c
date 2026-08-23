/*
 * APP - STM32F407VET6, RT-Thread
 * OLED + ESP8266 WiFi + terminal shell + SOS LED
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stm32f4xx.h>
#include "shell/shell.h"
#include "esp8266/esp8266.h"
#include "oled/OLED.h"
#include "wifi_config.h"

#define APP_FLASH_BASE  0x0800C000  /* must match APP1_BASE in partition.h */
#define LED0_PIN        GET_PIN(A, 1)

/* LED0 is active-low (PA1): LOW lights it */
#define LED_ON      PIN_LOW
#define LED_OFF     PIN_HIGH

/* LED blink timing (ms) */
#define SHORT_ON    100
#define SHORT_OFF   100
#define LONG_ON     300
#define LONG_OFF    200
#define CYCLE_PAUSE 500

/* LED mode: 0=blink, 1=on, 2=off */
static volatile int g_led_mode = 0;

static void led_blink(int on_ms, int off_ms, int times)
{
    for (int i = 0; i < times; i++)
    {
        if (g_led_mode != 0) return;
        rt_pin_write(LED0_PIN, LED_ON);
        rt_thread_mdelay(on_ms);
        if (g_led_mode != 0) return;
        rt_pin_write(LED0_PIN, LED_OFF);
        rt_thread_mdelay(off_ms);
    }
}

void shell_led_on(void)
{
    g_led_mode = 1;
    rt_pin_write(LED0_PIN, LED_ON);
}

void shell_led_off(void)
{
    g_led_mode = 2;
    rt_pin_write(LED0_PIN, LED_OFF);
}

void shell_led_blink(void)
{
    g_led_mode = 0;
}

int main(void)
{
    /*
     * Bootloader sets VTOR = 0x0800C000.  SystemInit() does NOT touch VTOR
     * on this BSP (USER_VECT_TAB_ADDRESS is not defined), so the bootloader's
     * setting persists.  Set it again here as a safeguard.
     */
    SCB->VTOR = APP_FLASH_BASE;

    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    /* init OLED display */
    OLED_Init();
    OLED_ShowString(1, 1, "WiFi: init...");

    /* load saved WiFi mode (default STA) + AP credentials */
    {
        int mode = WIFI_MODE_STA;
        char ssid[33] = "STM32-AP";
        char pass[65] = "12345678";
        if (wifi_mode_load(&mode, ssid, sizeof(ssid), pass, sizeof(pass)) == 0)
        {
            /* saved config loaded */
        }
        g_wifi_mode = mode;
        rt_strncpy(g_ap_ssid, ssid, 32); g_ap_ssid[32] = '\0';
        rt_strncpy(g_ap_pass, pass, 64); g_ap_pass[64] = '\0';
    }

    /* init ESP8266 WiFi (uart2 -> TCP Server port 23), STA or AP mode */
    esp8266_init();

    /* start terminal shell (uart1 + uart2) */
    shell_init();

    /* LED SOS blink loop */
    while (1)
    {
        if (g_led_mode == 0)
        {
            led_blink(SHORT_ON, SHORT_OFF, 3);  /* S */
            led_blink(LONG_ON, LONG_OFF, 3);    /* O */
            led_blink(SHORT_ON, SHORT_OFF, 3);  /* S */
            rt_thread_mdelay(CYCLE_PAUSE);
        }
        else
        {
            rt_thread_mdelay(100); /* idle when LED is static */
        }
    }
}
