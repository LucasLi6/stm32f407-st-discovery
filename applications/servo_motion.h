/*
 * Servo timed-motion engine — linear interpolation between angles.
 *
 * A dedicated thread advances 16 independent channels every 20 ms (the 50 Hz
 * servo refresh period), stepping each channel's PWM count linearly from its
 * current position to the target over the requested duration.
 *
 * All servo writes go through this module (a mutex serialises the software-I2C
 * so the motion thread can never interleave with a shell/script command).
 */

#ifndef __SERVO_MOTION_H__
#define __SERVO_MOTION_H__

#include <rtthread.h>

#define SERVO_MAX_DUR_MS    10000   /* max timed-move duration (0..10 s) */

int  servo_motion_init(void);

/* instant: jump to `deg` (0..180) */
void servo_motion_set(uint8_t n, uint16_t deg);

/* instant: cut PWM (keeps the tracked position for the next timed move) */
void servo_motion_off(uint8_t n);

/* timed: interpolate from current position to `deg` over `ms` (0 = instant).
 * Non-blocking — returns immediately.  Returns 0 on success, -1 if n invalid. */
int  servo_motion_move(uint8_t n, uint16_t deg, uint16_t ms);

/* 1 if channel n is still interpolating */
int  servo_motion_busy(uint8_t n);

/* cancel every channel and force all servos off (used by `stop`) */
void servo_motion_stop_all(void);

#endif /* __SERVO_MOTION_H__ */
