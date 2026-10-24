/*
 * Electric push rod (linear actuator) driver via 3 solid-state relays.
 *
 * H-bridge wiring:
 *   Relay 8 = power switch (ON  -> circuit closed / energized)
 *   Relay 6 = direction A  \
 *   Relay 7 = direction B  /   (6+7 = double-throw direction pair)
 *   DOWN = relay 8 ON, relay 6+7 OFF
 *   UP   = relay 8 ON, relay 6+7 ON
 *
 * Two modes:
 *   - Continuous (pushrod_set): just set the relay state, no timing.
 *   - Timed      (pushrod_move): a dedicated motion thread runs it for `ms`
 *     then cuts power; a timed move can be interrupted with pushrod_stop().
 */

#ifndef __PUSHROD_H__
#define __PUSHROD_H__

#include <rtthread.h>

#define PUSHROD_UP       1
#define PUSHROD_DOWN     0
#define PUSHROD_MAX_MS   20000   /* max timed move duration */
#define PUSHROD_END_MAX_MS 20000 /* safety timeout for a limit-switch move */

int  pushrod_init(void);                 /* create + start the motion thread */
int  pushrod_move(int dir, uint32_t ms); /* timed move (clamped to MAX_MS), -1 if busy */
int  pushrod_move_end(void);             /* DOWN until the limit switch (PC0) triggers, -1 if busy */
int  pushrod_busy(void);                 /* 1 while a timed move is running */
void pushrod_set(int dir);               /* continuous: set relay state directly (no busy) */
void pushrod_stop(void);                 /* interrupt a timed move (cut power now) */
void pushrod_off(void);                  /* all three relays off */

#endif
