/*
 * Minimal ESP8266 AT driver for TCP OTA.
 * USART2: PA2=TX PA3=RX 115200, interrupt RX with ring buffer.
 */
#ifndef __ESP8266_AT_H__
#define __ESP8266_AT_H__
#include <stdint.h>

void  esp_uart_init(void);
void  esp_send_str(const char *s);
int   esp_recv(char *buf, int max, int timeout_ms, int idle_ms);
void  esp_drain(void);

/* AT command, returns 0 if `expect` found in response (binary-safe). */
int   at_cmd(const char *cmd, const char *expect, int timeout_ms);

/* Wait for `expect` across multiple recv calls — for CWJAP/CIPSTART. */
int   at_cmd_wait(const char *cmd, const char *expect, int timeout_ms);

/* Enable/disable interrupt-driven RX ring buffer (for data phase). */
void  uart_irq_on(void);

/* Non-blocking read from ring buffer + DR (for transparent data phase). */
int   esp_poll(char *buf, int max);

/* Switch USART2 baud rate (APB1 = 36 MHz). */
void  esp_uart_set_9600(void);
void  esp_uart_set_115200(void);

#endif
