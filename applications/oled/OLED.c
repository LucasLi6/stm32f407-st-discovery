/*
 * SSD1306 OLED 0.96" 128x64 I2C driver
 * Software I2C: SCL=PB8, SDA=PB9
 * Adapted for RT-Thread STM32F103
 */

#include "OLED.h"
#include "OLED_Font.h"
#include <board.h>

/* -------- pin config -------- */
#define OLED_SCL_PIN  GET_PIN(B, 8)
#define OLED_SDA_PIN  GET_PIN(B, 9)

#define OLED_SCL(x)  rt_pin_write(OLED_SCL_PIN, (x))
#define OLED_SDA(x)  rt_pin_write(OLED_SDA_PIN, (x))

/* keep SCL/SDA transitions within the SSD1306 I2C spec (<=400kHz) */
static void i2c_delay(void)
{
    for (volatile uint32_t i = 0; i < 100; i++);
}

/* -------- software I2C -------- */
static void I2C_Init(void)
{
    rt_pin_mode(OLED_SCL_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_mode(OLED_SDA_PIN, PIN_MODE_OUTPUT_OD);
    OLED_SCL(1);
    OLED_SDA(1);
}

static void I2C_Start(void)
{
    OLED_SDA(1);
    i2c_delay();
    OLED_SCL(1);
    i2c_delay();     /* tSU;STA: hold SCL high before SDA falls */
    OLED_SDA(0);
    i2c_delay();     /* tHD;STA: hold SDA low before SCL falls */
    OLED_SCL(0);
    i2c_delay();
}

static void I2C_Stop(void)
{
    OLED_SDA(0);
    i2c_delay();
    OLED_SCL(1);
    i2c_delay();     /* tSU;STO: hold SCL high before SDA rises */
    OLED_SDA(1);
    i2c_delay();
}

static void I2C_SendByte(uint8_t Byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        OLED_SDA(!!(Byte & (0x80 >> i)));
        i2c_delay();
        OLED_SCL(1);
        i2c_delay();
        OLED_SCL(0);
        i2c_delay();
    }
    OLED_SCL(1);  /* extra clock, skip ACK */
    i2c_delay();
    OLED_SCL(0);
    i2c_delay();
}

/* -------- OLED command/data -------- */
static void WriteCommand(uint8_t cmd)
{
    I2C_Start();
    I2C_SendByte(0x78);   /* slave addr */
    I2C_SendByte(0x00);   /* command mode */
    I2C_SendByte(cmd);
    I2C_Stop();
}

static void WriteData(uint8_t dat)
{
    I2C_Start();
    I2C_SendByte(0x78);   /* slave addr */
    I2C_SendByte(0x40);   /* data mode */
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
    rt_thread_mdelay(50);

    I2C_Init();

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
