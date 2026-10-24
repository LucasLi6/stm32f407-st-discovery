/*
 * 8-channel relay driver on PE0..PE7 (active-low: pin LOW -> relay energized).
 */

#include "relay.h"
#include <rtdevice.h>
#include <board.h>

#define RELAY_PIN(i)  GET_PIN(E, (i))   /* i = 0..7 -> PE0..PE7 */

void relay_init(void)
{
    for (int i = 0; i < 8; i++)
    {
        rt_pin_mode(RELAY_PIN(i), PIN_MODE_OUTPUT);
        rt_pin_write(RELAY_PIN(i), RELAY_OFF);
    }

    /* CP 推杆 OFF 态 = RL4 开、RL5 关：上电即让 RL4 开启，CP 处于 OFF（安全）态 */
    relay_set(4, 1);
}

void relay_set(int num, int on)
{
    if (num < 1 || num > 8) return;
    rt_pin_write(RELAY_PIN(num - 1), on ? RELAY_ON : RELAY_OFF);
}
