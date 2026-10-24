/*
 * Terminal shell for STM32F103C8T6
 * Dual-UART: local (uart1) + remote ESP8266 (uart2)
 *
 * Architecture: 2 threads
 *   Reader — polls UART1+UART2, executes commands
 *   Writer — async CIPSEND output (blocks alone, reader never waits)
 */

#ifndef __SHELL_H__
#define __SHELL_H__

#include <rtthread.h>

#define SHELL_READER_STACK_SIZE   4096
#define SHELL_READER_PRIORITY     15
#define SHELL_WRITER_STACK_SIZE   1024
#define SHELL_WRITER_PRIORITY     14
#define SHELL_PROMPT              "STM32> "
#define SHELL_LINE_MAX            128

int shell_init(void);

/* write a string to the shell output (used by the script runner thread) */
void shell_put(const char *str);

#endif
