/*
 * SSD1306 OLED 0.96" 128x64 I2C driver — bare-metal port for bootloader
 *
 * Software (bit-bang) I2C on GPIOB: SCL=PB8, SDA=PB9.
 * No RTOS, no HAL — direct register access. Delay uses the bootloader's
 * SysTick millisecond counter (_ym_ms), so OLED_Init() must be called
 * after clock_init().
 */

#include "oled.h"
#include "oled_font.h"
#include <stm32f4xx.h>

extern volatile uint32_t _ym_ms;   /* 1 ms tick, driven by SysTick_Handler */

/* -------- pin config -------- */
#define OLED_SCL_PIN  8
#define OLED_SDA_PIN  9

#define SCL_H()  (GPIOB->BSRR = (1UL << OLED_SCL_PIN))
#define SCL_L()  (GPIOB->BSRR = (1UL << OLED_SCL_PIN) << 16)
#define SDA_H()  (GPIOB->BSRR = (1UL << OLED_SDA_PIN))
#define SDA_L()  (GPIOB->BSRR = (1UL << OLED_SDA_PIN) << 16)

static void oled_delay_ms(uint32_t ms)
{
    uint32_t s = _ym_ms;
    while ((_ym_ms - s) < ms);
}

/* keep SCL/SDA transitions within the SSD1306 I2C spec (≤400 kHz) */
static void i2c_delay(void)
{
    for (volatile uint32_t i = 0; i < 100; i++);
}

/* -------- software I2C -------- */
static void I2C_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* PB8/PB9: open-drain output — matches the proven F103 reference driver.
     * Open-drain releases the line for a high level and lets the OLED module's
     * own pull-ups (or our internal pull-ups) pull it up, so the slave can
     * drive SDA low for ACK without fighting a push-pull master.
     */
    GPIOB->MODER  &= ~((3UL << (OLED_SCL_PIN * 2)) | (3UL << (OLED_SDA_PIN * 2)));
    GPIOB->MODER  |=  ((1UL << (OLED_SCL_PIN * 2)) | (1UL << (OLED_SDA_PIN * 2)));
    GPIOB->OTYPER |=  ((1UL << OLED_SCL_PIN) | (1UL << OLED_SDA_PIN));  /* open-drain */
    GPIOB->PUPDR  &= ~((3UL << (OLED_SCL_PIN * 2)) | (3UL << (OLED_SDA_PIN * 2)));
    GPIOB->PUPDR  |=  ((1UL << (OLED_SCL_PIN * 2)) | (1UL << (OLED_SDA_PIN * 2)));  /* internal pull-up */

    SCL_H();
    SDA_H();
}

static void I2C_Start(void)
{
    SDA_H();
    i2c_delay();
    SCL_H();
    i2c_delay();     /* tSU;STA: hold SCL high before SDA falls (>=0.6us) */
    SDA_L();
    i2c_delay();     /* tHD;STA: hold SDA low before SCL falls */
    SCL_L();
    i2c_delay();
}

static void I2C_Stop(void)
{
    SDA_L();
    i2c_delay();
    SCL_H();
    i2c_delay();     /* tSU;STO: hold SCL high before SDA rises */
    SDA_H();
    i2c_delay();
}

static void I2C_SendByte(uint8_t Byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (Byte & (0x80 >> i)) SDA_H(); else SDA_L();
        i2c_delay();
        SCL_H();
        i2c_delay();
        SCL_L();
        i2c_delay();
    }
    SCL_H();  /* extra clock, skip ACK */
    i2c_delay();
    SCL_L();
    i2c_delay();
}

/* -------- slave address (0x78 with SA0=0, 0x7A with SA0=1) --------
 * Deliberately uninitialised (bss): this bootloader's Reset_Handler does
 * NOT copy the .data section, so a `= 0x78` initialiser would read back as
 * garbage. oled_addr is assigned at runtime in OLED_Init().
 */
static uint8_t oled_addr;
volatile int oled_probe_result;   /* 0 = none, 1 = 0x78 ACK, 2 = 0x7A ACK */

/* -------- ACK probe (diagnostics) -------- */
static void sda_dir_input(void)
{
    GPIOB->MODER &= ~(3UL << (OLED_SDA_PIN * 2));                  /* input */
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(3UL << (OLED_SDA_PIN * 2)))
                 |  (1UL << (OLED_SDA_PIN * 2));                   /* pull-up */
}
static void sda_dir_output(void)
{
    GPIOB->MODER = (GPIOB->MODER & ~(3UL << (OLED_SDA_PIN * 2)))
                 |  (1UL << (OLED_SDA_PIN * 2));                      /* output */
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(3UL << (OLED_SDA_PIN * 2)))
                 |  (1UL << (OLED_SDA_PIN * 2));                      /* keep pull-up (open-drain) */
}

/* send 8 bits MSB-first, SCL ends low, no ACK clock */
static void i2c_send8(uint8_t b)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (b & (0x80 >> i)) SDA_H(); else SDA_L();
        i2c_delay();
        SCL_H(); i2c_delay();
        SCL_L(); i2c_delay();
    }
}

/* clock one ACK bit; returns 1 if SDA low (ACK), 0 if high (NACK) */
static int i2c_read_ack(void)
{
    int ack;
    sda_dir_input();      /* release SDA; pull-up holds it high unless slave ACKs */
    i2c_delay();
    SCL_H();              /* 9th clock */
    i2c_delay();
    ack = ((GPIOB->IDR & (1UL << OLED_SDA_PIN)) == 0);   /* low = ACK */
    SCL_L();
    i2c_delay();
    sda_dir_output();
    SDA_H();
    return ack;
}

int OLED_Probe(void)
{
    oled_probe_result = 0;

    I2C_Start();
    i2c_send8(0x78);
    if (i2c_read_ack()) { oled_addr = 0x78; oled_probe_result = 1; }
    I2C_Stop();

    if (!oled_probe_result)
    {
        I2C_Start();
        i2c_send8(0x7A);
        if (i2c_read_ack()) { oled_addr = 0x7A; oled_probe_result = 2; }
        I2C_Stop();
    }

    return oled_probe_result;
}

/* -------- OLED command/data -------- */
static void WriteCommand(uint8_t cmd)
{
    I2C_Start();
    I2C_SendByte(oled_addr);   /* slave addr */
    I2C_SendByte(0x00);        /* command mode */
    I2C_SendByte(cmd);
    I2C_Stop();
}

static void WriteData(uint8_t dat)
{
    I2C_Start();
    I2C_SendByte(oled_addr);   /* slave addr */
    I2C_SendByte(0x40);        /* data mode */
    I2C_SendByte(dat);
    I2C_Stop();
}

static void SetCursor(uint8_t Y, uint8_t X)
{
    WriteCommand(0xB0 | Y);
    WriteCommand(0x10 | ((X & 0xF0) >> 4));
    WriteCommand(0x00 | (X & 0x0F));
}

/* -------- public API -------- */
void OLED_Clear(void)
{
    for (uint8_t j = 0; j < 8; j++)
    {
        SetCursor(j, 0);
        for (uint8_t i = 0; i < 128; i++)
            WriteData(0x00);
    }
}

void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (uint8_t i = 0; i < 8; i++)
        WriteData(OLED_F8x16[Char - ' '][i]);
    SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (uint8_t i = 0; i < 8; i++)
        WriteData(OLED_F8x16[Char - ' '][i + 8]);
}

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    for (uint8_t i = 0; String[i] != '\0'; i++)
        OLED_ShowChar(Line, Column + i, String[i]);
}

static uint32_t Pow(uint32_t X, uint32_t Y)
{
    uint32_t r = 1;
    while (Y--) r *= X;
    return r;
}

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i, Number / Pow(10, Length - i - 1) % 10 + '0');
}

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        for (uint8_t i = 0; i < Length; i++)
            OLED_ShowChar(Line, Column + i + 1, Number / Pow(10, Length - i - 1) % 10 + '0');
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number = -Number;
        for (uint8_t i = 0; i < Length; i++)
            OLED_ShowChar(Line, Column + i + 1, Number / Pow(10, Length - i - 1) % 10 + '0');
    }
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++)
    {
        uint8_t n = Number / Pow(16, Length - i - 1) % 16;
        OLED_ShowChar(Line, Column + i, n < 10 ? n + '0' : n - 10 + 'A');
    }
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i, Number / Pow(2, Length - i - 1) % 2 + '0');
}

void OLED_Init(void)
{
    /* power-up delay */
    oled_delay_ms(50);

    I2C_Init();

    /* detect slave address (0x78 / 0x7A) before sending init commands */
    oled_addr = 0x78;
    OLED_Probe();

    WriteCommand(0xAE);  /* display off */
    WriteCommand(0xD5); WriteCommand(0x80);  /* clock */
    WriteCommand(0xA8); WriteCommand(0x3F);  /* multiplex */
    WriteCommand(0xD3); WriteCommand(0x00);  /* offset */
    WriteCommand(0x40);  /* start line */
    WriteCommand(0xA1);  /* segment remap */
    WriteCommand(0xC8);  /* COM scan dir */
    WriteCommand(0xDA); WriteCommand(0x12);  /* COM pins */
    WriteCommand(0x81); WriteCommand(0xCF);  /* contrast */
    WriteCommand(0xD9); WriteCommand(0xF1);  /* pre-charge */
    WriteCommand(0xDB); WriteCommand(0x30);  /* VCOM */
    WriteCommand(0xA4);  /* entire display */
    WriteCommand(0xA6);  /* normal */
    WriteCommand(0x8D); WriteCommand(0x14);  /* charge pump */
    WriteCommand(0xAF);  /* display on */

    OLED_Clear();
}
