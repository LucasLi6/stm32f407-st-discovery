/*
 * SSD1306 OLED 0.96" 128x64 I2C driver — bare-metal port for bootloader
 * Software I2C: SCL=PB8, SDA=PB9
 */

#ifndef __OLED_H__
#define __OLED_H__

#include <stdint.h>

void OLED_Init(void);
int  OLED_Probe(void);
extern volatile int oled_probe_result;   /* 0 none, 1 = 0x78 ACK, 2 = 0x7A ACK */
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif
