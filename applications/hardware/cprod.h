/*
 * Second electric push rod ("CP") on solid-state relays 4,5.
 *
 * 2-relay reversing pair (no separate power switch):
 *   RL4 = direction A, RL5 = direction B.
 *   UP   = RL4 OFF, RL5 OFF
 *   DOWN = RL4 ON,  RL5 ON
 *   OFF  = RL4 ON,  RL5 OFF   (safe / de-energised state)
 *
 * Two modes (same control logic as pushrod.h):
 *   - Continuous (cprod_set): just set the relay state, no timing.
 *   - Timed      (cprod_move): a dedicated motion thread runs it for `ms`
 *     then returns to OFF; can be interrupted with cprod_stop().
 */

#ifndef __CPROD_H__
#define __CPROD_H__

#include <rtthread.h>

#define CPROD_UP       1
#define CPROD_DOWN     0
#define CPROD_MAX_MS   20000   /* max timed move duration */

int  cprod_init(void);                 /* create + start the motion thread */
int  cprod_move(int dir, uint32_t ms); /* timed move (clamped to MAX_MS), -1 if busy */
int  cprod_busy(void);                 /* 1 while a timed move is running */
void cprod_set(int dir);               /* continuous: set relay state directly (no busy) */
void cprod_stop(void);                 /* interrupt a timed move (return to OFF) */
void cprod_off(void);                  /* RL4 ON, RL5 OFF */

#endif
