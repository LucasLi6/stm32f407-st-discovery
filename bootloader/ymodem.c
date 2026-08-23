/*
 * Minimal Ymodem receiver — 128B packets, CRC-16, polling UART1.
 * Compatible with TeraTerm / SecureCRT / XShell.
 */

#include "ymodem.h"
#include "flash_ops.h"
#include "partition.h"
#include "ota_config.h"
#include <stm32f4xx.h>

void ymodem_uart_init(void)
{
    /* F4: GPIO clock on AHB1, USART1 on APB2 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* PA9 = USART1_TX (AF7), PA10 = USART1_RX (AF7) */
    GPIOA->MODER   &= ~((3 << 18) | (3 << 20));
    GPIOA->MODER   |=  ((2 << 18) | (2 << 20));  /* alternate function */
    GPIOA->AFR[1]  &= ~((0xF << 4) | (0xF << 8));
    GPIOA->AFR[1]  |=  ((7 << 4) | (7 << 8));    /* AF7 */
    GPIOA->PUPDR   &= ~(3 << 20);
    GPIOA->PUPDR   |=  (1 << 20);                 /* PA10 pull-up */
    GPIOA->OSPEEDR |=  (3 << 18) | (3 << 20);     /* high speed */

    USART1->BRR = (45 << 4) | 9;     /* 115200 @ 84MHz APB2 */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

int ymodem_putc(int c)
{
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (uint8_t)c;
    return c;
}

extern volatile uint32_t _ym_ms;  /* SysTick counter from main.c */
int ymodem_getc(int timeout_ms)
{
    uint32_t start = _ym_ms;
    while (1) {
        if (USART1->SR & USART_SR_RXNE)
            return (int)(USART1->DR & 0xFF);
        if ((_ym_ms - start) >= (uint32_t)timeout_ms) return -1;
    }
}

uint16_t crc16(const uint8_t *d, int n)
{
    uint16_t c = 0;
    for (int i = 0; i < n; i++) {
        c ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++)
            c = (c & 0x8000) ? (c << 1) ^ 0x1021 : (c << 1);
    }
    return c;
}

#define SOH 0x01
#define STX 0x02
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CAN   0x18
#define CRC16  'C'
#define DLEN 128
#define PLEN (DLEN+5)

/*
 * Wait for SOH/STX/EOT.  Returns byte or -1 on timeout.
 */
static int wait_hdr(int timeout_ms)
{
    while (1) {
        int c = ymodem_getc(timeout_ms);
        if (c < 0) return -1;
        if (c == SOH || c == STX || c == EOT) return c;
    }
}

/*
 * Read one Ymodem packet.  Assumes SOH/STX already consumed.
 * Returns: data length on success, 0 for empty final packet, -1 error, -2 CRC.
 */
static int read_pkt(int typ, uint8_t *out, int *seq_out)
{
    int dlen = (typ == SOH) ? 128 : 1024;
    uint8_t raw[PLEN];
    raw[0] = (uint8_t)typ;
    for (int i = 1; i < dlen + 5; i++) {
        int c = ymodem_getc(3000);
        if (c < 0) return -1;
        raw[i] = (uint8_t)c;
    }
    int seq = raw[1];
    if ((seq ^ raw[2]) != 0xFF) return -1;

    if (seq == 0) {
        *seq_out = 0;
        return 0;   /* final empty packet */
    }

    uint16_t calc = crc16(&raw[3], dlen);
    uint16_t recv = ((uint16_t)raw[3+dlen] << 8) | raw[4+dlen];
    if (calc != recv) return -2;

    for (int i = 0; i < dlen; i++) out[i] = raw[3+i];
    *seq_out = seq;
    return dlen;
}

int ymodem_receive(uint32_t flash_addr, uint32_t max_size)
{
    int offset = 0, first = 1;
    uint8_t buf[1024];

    /*
     * Phase 1: send 'C' until sender responds with a packet.
     * Give up after OTA_FIRST_PKT_TIMEOUT (15s) with no first packet.
     */
    uint32_t start = _ym_ms;
    while (1) {
        if ((_ym_ms - start) >= OTA_FIRST_PKT_TIMEOUT) return -1;  /* 15s no first packet */
        ymodem_putc(CRC16);
        int h = wait_hdr(1000);
        if (h == SOH || h == STX) {
            int seq, n = read_pkt(h, buf, &seq);
            if (n < 0) { ymodem_putc(NAK); continue; }   // blink: bad pkt
            if (n == 0) { ymodem_putc(ACK); continue; }  // blink: final
            if (offset + n <= (int)max_size) {
                flash_write_bytes(flash_addr + offset, buf, n);
                offset += n;
            }
            ymodem_putc(ACK);                             // blink: wrote+acked
            (void)seq;
            first = 0;
            break;
        }
    }
    if (first) return -1;  /* never got a packet */

    /*
     * Phase 2: receive remaining data packets.
     */
    while (1) {
        int h = wait_hdr(60000);  /* 60s between packets */
        if (h < 0) return -1;

        if (h == EOT) {
            ymodem_putc(NAK);
            h = wait_hdr(10000);
            if (h == EOT) {
                ymodem_putc(ACK);
                ymodem_putc(CRC16);
                /* wait for empty final packet */
                h = wait_hdr(10000);
                if (h == SOH) {
                    int seq, n = read_pkt(h, buf, &seq);
                    if (n == 0) ymodem_putc(ACK);
                }
                return offset;
            }
            continue;
        }

        if (h == SOH || h == STX) {
            int seq, n = read_pkt(h, buf, &seq);
            if (n < 0) { ymodem_putc(NAK); continue; }
            if (n == 0) { ymodem_putc(ACK); continue; }
            if (offset + n <= (int)max_size) {
                flash_write_bytes(flash_addr + offset, buf, n);
                offset += n;
            }
            ymodem_putc(ACK);
            (void)seq;
        }
    }
}
