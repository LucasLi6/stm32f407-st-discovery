/*
 * PCA9685 16-channel PWM / servo driver
 * Software I2C on PB6 (SCL) / PB7 (SDA), mirroring the OLED bit-bang style.
 * Ported from the F103 robot-dog code (Hardware/PCA9685.c + User/myiic.c).
 */

#include "pca9685.h"
#include <rtdevice.h>
#include <board.h>

/* ---- pin config ---- */
#define PCA_SCL_PIN   GET_PIN(B, 6)
#define PCA_SDA_PIN   GET_PIN(B, 7)

#define PCA_SCL(x)    rt_pin_write(PCA_SCL_PIN, (x))
#define PCA_SDA(x)    rt_pin_write(PCA_SDA_PIN, (x))

/* keep transitions within the PCA9685 I2C spec (<=1 MHz) */
static void i2c_delay(void)
{
    for (volatile uint32_t i = 0; i < 100; i++);
}

/* ---- software I2C primitives ---- */
static void i2c_start(void)
{
    PCA_SDA(1);
    i2c_delay();
    PCA_SCL(1);
    i2c_delay();
    PCA_SDA(0);      /* SDA falls while SCL high -> START */
    i2c_delay();
    PCA_SCL(0);
    i2c_delay();
}

static void i2c_stop(void)
{
    PCA_SDA(0);
    i2c_delay();
    PCA_SCL(1);
    i2c_delay();
    PCA_SDA(1);      /* SDA rises while SCL high -> STOP */
    i2c_delay();
}

/* Shift out one byte, then clock the 9th bit to read ACK.
 * Returns 0 on ACK (SDA pulled low), -1 on NACK. */
static int i2c_write_byte(uint8_t byte)
{
    for (int i = 7; i >= 0; i--)
    {
        PCA_SDA((byte >> i) & 1);
        i2c_delay();
        PCA_SCL(1);
        i2c_delay();
        PCA_SCL(0);
        i2c_delay();
    }
    /* 9th clock: release SDA (open-drain) and read slave's ACK */
    PCA_SDA(1);
    i2c_delay();
    PCA_SCL(1);
    i2c_delay();
    int ack = (rt_pin_read(PCA_SDA_PIN) == 0) ? 0 : -1;
    PCA_SCL(0);
    i2c_delay();
    return ack;
}

/* ---- register write (aborts silently on NACK) ---- */
static int pca_write_reg(uint8_t reg, uint8_t val)
{
    i2c_start();
    if (i2c_write_byte((uint8_t)(PCA9685_ADDR << 1)) != 0) { i2c_stop(); return -1; }
    if (i2c_write_byte(reg)                          != 0) { i2c_stop(); return -1; }
    if (i2c_write_byte(val)                          != 0) { i2c_stop(); return -1; }
    i2c_stop();
    return 0;
}

/* prescale = round(25 MHz / (4096 * freq)) - 1  (internal 25 MHz oscillator) */
static void pca_set_freq(uint32_t freq)
{
    uint32_t prescale = (25000000UL + (4096UL * freq) / 2) / (4096UL * freq);
    if (prescale > 0) prescale -= 1;

    pca_write_reg(PCA9685_MODE1, 0x10);             /* sleep */
    pca_write_reg(PCA9685_PRESCALE, (uint8_t)prescale);
    pca_write_reg(PCA9685_MODE1, 0xA1);             /* auto-inc + restart */
    rt_thread_mdelay(2);
}

/* ---- public API ---- */
void pca9685_set_pwm(uint8_t num, uint16_t on, uint16_t off)
{
    if (num > 15) return;
    uint8_t reg = PCA9685_LED0_ON_L + 4 * num;
    pca_write_reg(reg,     (uint8_t)(on  & 0xFF));
    pca_write_reg(reg + 1, (uint8_t)(on  >> 8));
    pca_write_reg(reg + 2, (uint8_t)(off & 0xFF));
    pca_write_reg(reg + 3, (uint8_t)(off >> 8));
}

void servo_set_angle(uint8_t num, uint16_t deg)
{
    if (num > 15) return;
    if (deg > PCA9685_SERVO_MAX_DEG) deg = PCA9685_SERVO_MAX_DEG;

    int32_t off = PCA9685_SERVO_CENTER
                + ((int32_t)deg - PCA9685_SERVO_NEUTRAL_DEG)
                  * PCA9685_SERVO_SLOPE_N / PCA9685_SERVO_SLOPE_D;
    if (off < 0)    off = 0;
    if (off > 4095) off = 4095;

    pca9685_set_pwm(num, 0, (uint16_t)off);
}

void servo_set_off(uint8_t num)
{
    if (num > 15) return;
    /* OFF = 0x1000 sets LEDn_OFF "full-off" bit -> channel driven low, no PWM */
    pca9685_set_pwm(num, 0, 0x1000);
}

int pca9685_init(void)
{
    rt_pin_mode(PCA_SCL_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_mode(PCA_SDA_PIN, PIN_MODE_OUTPUT_OD);
    PCA_SCL(1);
    PCA_SDA(1);
    rt_thread_mdelay(10);

    if (pca_write_reg(PCA9685_MODE1, 0x00) != 0)   /* reset + presence check */
        return -1;
    pca_write_reg(PCA9685_MODE2, 0x04);            /* totem-pole outputs */
    pca_set_freq(PCA9685_SERVO_FREQ);

    for (uint8_t i = 0; i < 16; i++)
        servo_set_off(i);                          /* start with no PWM output */
    return 0;
}
