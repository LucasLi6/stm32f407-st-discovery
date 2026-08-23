/*
 * Minimal Ymodem receiver — 128-byte packets, CRC-16, pure polling.
 * Protocol: TeraTerm / SecureCRT / XShell compatible.
 */

#ifndef __YMODEM_H__
#define __YMODEM_H__

#include <stdint.h>

/* Init USART1 (PA9=TX PA10=RX 115200) for Ymodem communication. */
void ymodem_uart_init(void);

int  ymodem_putc(int c);
int  ymodem_getc(int timeout_ms);
uint16_t crc16(const uint8_t *d, int n);

/* Receive firmware via Ymodem, write to flash at `addr`.
 * Returns number of bytes written, or -1 on error. */
int  ymodem_receive(uint32_t flash_addr, uint32_t max_size);

#endif
