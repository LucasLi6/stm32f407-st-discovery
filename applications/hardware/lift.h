/*
 * Lift platform (升降台) on solid-state relays 2,3.
 *
 * Each relay switches one motor terminal between GND (relay off) and +V (on):
 *   RL2 = up   relay,  RL3 = down relay
 *   UP   = RL2 ON,  RL3 OFF
 *   DN   = RL2 OFF, RL3 ON
 *   OFF  = RL2 OFF, RL3 OFF   (both motor terminals grounded -> stopped)
 *
 * Limit switches (active-low, S -> GND, pull-up inputs):
 *   bottom = S2: stops a DN move
 *   top    = S3: stops an UP move (reserved, not yet wired)
 *
 * Every move — timed or "until limit" — polls the limit switch for its own
 * direction and stops immediately when it triggers, even if time remains.
 */

#ifndef __LIFT_H__
#define __LIFT_H__

#include <rtthread.h>

#define LIFT_UP         1
#define LIFT_DN         0
#define LIFT_MAX_MS     20000   /* max timed move duration */
#define LIFT_END_MAX_MS 20000   /* safety timeout for an "until limit" move */

int  lift_init(void);                    /* create + start the motion thread */
int  lift_move(int dir, uint32_t ms);    /* timed move (clamped), stops early on limit; -1 if busy */
int  lift_move_end(int dir);             /* move until the direction's limit triggers; -1 if busy */
int  lift_busy(void);                    /* 1 while a move is running */
void lift_stop(void);                    /* interrupt a move (both relays off) */
void lift_off(void);                     /* both relays off (stopped) */

#endif
