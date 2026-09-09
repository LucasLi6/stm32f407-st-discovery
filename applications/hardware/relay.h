/*
 * 8-channel relay driver on PE0..PE7 (active-low: pin LOW -> relay energized).
 */

#ifndef __RELAY_H__
#define __RELAY_H__

#include <rtthread.h>

#define RELAY_ON    PIN_LOW    /* active-low: pin LOW -> relay energized */
#define RELAY_OFF   PIN_HIGH

void relay_init(void);
void relay_set(int num, int on);    /* num 1-8, on 1/0 */

#endif
