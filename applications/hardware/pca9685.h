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

/* ---- servo pulse mapping (50 Hz, 4096 counts = 20 ms) ----
 * off = PULSE_0 + angle * SLOPE_N / SLOPE_D  (0-180 deg)
 *
 * 360-deg CONTINUOUS-ROTATION servos: "angle" is a speed/direction command.
 * NEUTRAL_DEG (90) is the stop pulse; below it the servo spins one way
 * (faster toward 0), above it the other way. */
#define PCA9685_SERVO_FREQ      50
#define PCA9685_SERVO_PULSE_0   136          /* counts @ 0 deg (~0.66 ms) */
#define PCA9685_SERVO_SLOPE_N   22           /* counts per deg = 2.2 */
#define PCA9685_SERVO_SLOPE_D   10
#define PCA9685_SERVO_MAX_DEG   180          /* clamp for speed command 0..180 */
#define PCA9685_SERVO_NEUTRAL_DEG 90        /* 360 servo stop pulse (~1.63 ms) */

int  pca9685_init(void);                    /* 50 Hz, stop all servos at NEUTRAL_DEG */
void pca9685_set_pwm(uint8_t num, uint16_t on, uint16_t off);
void servo_set_angle(uint8_t num, uint16_t deg);   /* num 0-15, deg 0..MAX */

#endif
