/*
 * APP LED SOS — STM32F407VET6, RT-Thread
 * One thread: blinks "S O S" in Morse code on PA1 (active-low).
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stm32f4xx.h>

#define APP_FLASH_BASE  0x08005000  /* must match APP_BASE in partition.h */
#define LED0_PIN        GET_PIN(A, 1)

/* Morse timing unit (ms). dot=1 unit, dash=3 units, letter gap=3, word gap=7 */
#define DOT_MS  50

static void led_on(void)  { rt_pin_write(LED0_PIN, PIN_LOW);  } /* active-low: LOW lights it */
static void led_off(void) { rt_pin_write(LED0_PIN, PIN_HIGH); }

static void s_dot(void)  { led_on(); rt_thread_mdelay(DOT_MS);     led_off(); }
static void s_dash(void) { led_on(); rt_thread_mdelay(3 * DOT_MS); led_off(); }

static void led_thread_entry(void *parameter)
{
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    while (1)
    {
        /* S = ... */
        s_dot(); rt_thread_mdelay(DOT_MS);
        s_dot(); rt_thread_mdelay(DOT_MS);
        s_dot(); rt_thread_mdelay(3 * DOT_MS);   /* letter gap */

        /* O = --- */
        s_dash(); rt_thread_mdelay(DOT_MS);
        s_dash(); rt_thread_mdelay(DOT_MS);
        s_dash(); rt_thread_mdelay(3 * DOT_MS);  /* letter gap */

        /* S = ... */
        s_dot(); rt_thread_mdelay(DOT_MS);
        s_dot(); rt_thread_mdelay(DOT_MS);
        s_dot();

        /* word gap before repeating the whole SOS */
        rt_thread_mdelay(7 * DOT_MS);
    }
}

int main(void)
{
    /*
     * Bootloader sets VTOR = 0x08005000.  SystemInit() does NOT touch
     * VTOR on this BSP (USER_VECT_TAB_ADDRESS is not defined), so the
     * bootloader's setting persists.  Set it again here as a safeguard.
     */
    SCB->VTOR = APP_FLASH_BASE;

    rt_thread_t tid = rt_thread_create(
        "led",
        led_thread_entry,
        RT_NULL,
        512,
        RT_THREAD_PRIORITY_MAX / 3,
        20
    );

    if (tid != RT_NULL)
        rt_thread_startup(tid);

    return 0;
}
