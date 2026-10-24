/*
 * PCA9685 16-channel PWM / servo driver
 * Software I2C: SCL=PB6, SDA=PB7
 * Ported from the F103 robot-dog code (Hardware/PCA9685.c) to RT-Thread F407.
 */

#ifndef __PCA9685_H__
#define __PCA9685_H__

#include <rtthread.h>

/* ---- I2C address (7-bit, A0-A5 all low) ---- */
#define PCA9685_ADDR            0x40

/* ---- registers ---- */
#define PCA9685_MODE1           0x00
#define PCA9685_MODE2           0x01
#define PCA9685_PRESCALE        0xFE
#define PCA9685_LED0_ON_L       0x06

/* ---- servo pulse mapping (50 Hz, 4096 counts = 20 ms -> 1 count = 4.883 us) ----
 * off = CENTER + (deg - NEUTRAL_DEG) * SLOPE_N / SLOPE_D   (deg 0..180)
 *
 * The centre is pinned at the servo's measured neutral (1.63 ms = 334 counts)
 * and the pulse swings symmetrically around it out to 0.46 ms / 2.80 ms so the
 * servo actually reaches both mechanical end-stops.  If a servo stalls
 * (buzzing / running hot) before the pulse reaches its extreme, pull MIN/MAX in. */
#define PCA9685_SERVO_FREQ      50
#define PCA9685_SERVO_NEUTRAL_DEG 90         /* angle (deg) at servo centre */
#define PCA9685_SERVO_CENTER    334          /* counts @ centre (1.63 ms) */
#define PCA9685_SERVO_SLOPE_N   8            /* counts per deg = 8/3 (240 over 90 deg) */
#define PCA9685_SERVO_SLOPE_D   3
#define PCA9685_SERVO_MIN_COUNT 94           /* 0 deg   -> 0.46 ms */
#define PCA9685_SERVO_MAX_COUNT 574          /* 180 deg -> 2.80 ms */
#define PCA9685_SERVO_MAX_DEG   180          /* clamp 0..180 */

int  pca9685_init(void);                    /* 50 Hz, all channels off (no PWM) */
void pca9685_set_pwm(uint8_t num, uint16_t on, uint16_t off);
void servo_set_angle(uint8_t num, uint16_t deg);   /* num 0-15, deg 0..MAX */
void servo_set_off(uint8_t num);             /* force channel off (no PWM output) */

#endif
