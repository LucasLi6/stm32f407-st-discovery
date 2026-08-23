/*
 * Minimal ESP8266 AT driver — USART2, interrupt RX ring buffer, binary-safe.
 */
#include "esp8266_at.h"
#include <stm32f4xx.h>
#include <string.h>

extern volatile uint32_t _ym_ms;

/* ---- ring buffer ---- */
#define RB_SZ 8192
volatile uint8_t _rb[RB_SZ];
volatile int _rb_w, _rb_r, _irq_on;

void USART2_IRQHandler(void)
{
    if (USART2->SR & USART_SR_RXNE) {
        uint8_t b = (uint8_t)(USART2->DR & 0xFF);
        int nxt = (_rb_w + 1) % RB_SZ;
        if (nxt != _rb_r) { _rb[_rb_w] = b; _rb_w = nxt; }
    }
}
static int rb_read(char *d, int max) { int n=0; __disable_irq(); while(n<max&&_rb_r!=_rb_w){d[n++]=(char)_rb[_rb_r];_rb_r=(_rb_r+1)%RB_SZ;} __enable_irq(); return n; }

void uart_irq_on(void)
{
    _irq_on = 1;
    USART2->CR1 |= USART_CR1_RXNEIE;
    NVIC_SetPriority(USART2_IRQn, 0);
    NVIC_EnableIRQ(USART2_IRQn);
}

/* ---- UART init ---- */
static void _dbg(char c){ while(!(USART1->SR&USART_SR_TXE)); USART1->DR=c; }
static void _dbg_s(const char *s){ while(*s)_dbg(*s++); }
#define DBG(s) _dbg_s(s"\r\n")

void esp_uart_init(void)
{
    /* F4: GPIO clock on AHB1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA9 = USART1_TX (AF7, debug output) */
    GPIOA->MODER  &= ~(3 << 18);
    GPIOA->MODER  |=  (2 << 18);              /* alternate function */
    GPIOA->AFR[1] &= ~(0xF << 4);
    GPIOA->AFR[1] |=  (7 << 4);              /* AF7 */
    GPIOA->OSPEEDR |= (3 << 18);              /* high speed */
    USART1->BRR = (45 << 4) | 9;             /* 115200 @ 84MHz APB2 */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;

    /* PA2 = USART2_TX, PA3 = USART2_RX (AF7) */
    GPIOA->MODER  &= ~((3 << 4) | (3 << 6));
    GPIOA->MODER  |=  ((2 << 4) | (2 << 6));  /* alternate function */
    GPIOA->AFR[0] &= ~((0xF << 8) | (0xF << 12));
    GPIOA->AFR[0] |=  ((7 << 8) | (7 << 12)); /* AF7 */
    GPIOA->OSPEEDR |= (3 << 4) | (3 << 6);    /* high speed */
    USART2->BRR = (22 << 4) | 13;            /* 115200 @ 42MHz APB1 */
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void esp_send_str(const char *s) {
    while (*s) { while (!(USART2->SR & USART_SR_TXE)); USART2->DR = *s++; }
}

void esp_drain(void) { char d[64]; if(_irq_on) while(rb_read(d,sizeof(d))>0); while(USART2->SR&USART_SR_RXNE)(void)(USART2->DR&0xFF); }

/* Non-blocking: read whatever is available now from ring buffer + DR. */
int esp_poll(char *buf, int max)
{
    int n = 0;
    if (_irq_on) n = rb_read(buf, max);
    while (n < max && (USART2->SR & USART_SR_RXNE))
        buf[n++] = (char)(USART2->DR & 0xFF);
    return n;
}

/* ---- baud-rate switch (APB1 = 42 MHz on F407) ---- */
void esp_uart_set_9600(void)
{
    USART2->CR1 &= ~USART_CR1_UE;
    /* 42M / (16 * 9600) = 273.4375  →  mantissa=273, fraction=7 */
    USART2->BRR = (273 << 4) | 7;
    USART2->CR1 |= USART_CR1_UE;
}

void esp_uart_set_115200(void)
{
    USART2->CR1 &= ~USART_CR1_UE;
    /* 42M / (16 * 115200) = 22.786  →  mantissa=22, fraction=13 */
    USART2->BRR = (22 << 4) | 13;
    USART2->CR1 |= USART_CR1_UE;
}

int esp_recv(char *b, int m, int to_ms, int idle) {
    int t=0; uint32_t ts=_ym_ms, tl=_ym_ms;
    while(t<m-1){
        /* check ring buffer first (ISR puts data here) */
        if(_irq_on){ int n=rb_read(b+t,m-t-1); if(n>0){t+=n;tl=_ym_ms;continue;} }
        /* then check DR directly */
        if(USART2->SR&USART_SR_RXNE){b[t++]=(char)(USART2->DR&0xFF);tl=_ym_ms;continue;}
        if(t==0&&(_ym_ms-ts)>=(uint32_t)to_ms)break;
        if(t>0&&(_ym_ms-tl)>=(uint32_t)idle)break;
    }
    b[t]=0; return t;
}

/* binary-safe strstr */
static char *b_strstr(const char *h, int hl, const char *n) {
    int nl=strlen(n);
    for(int i=0;i<=hl-nl;i++) if(memcmp(h+i,n,nl)==0) return (char*)(h+i);
    return 0;
}

int at_cmd(const char *cmd, const char *expect, int to) {
    char buf[512]; esp_drain();
    if(cmd) esp_send_str(cmd);
    int n=esp_recv(buf,sizeof(buf),to,100);
    return b_strstr(buf,n,expect)?0:-1;
}

int at_cmd_wait(const char *cmd, const char *expect, int to) {
    char buf[512]; int t=0; uint32_t dl=_ym_ms+to;
    esp_drain(); if(cmd) esp_send_str(cmd);
    while(_ym_ms<dl&&t<(int)sizeof(buf)-1){
        int n=esp_recv(buf+t,sizeof(buf)-t-1,2000,2000);
        if(n>0){t+=n;buf[t]=0;if(b_strstr(buf,t,expect))return 0;}
    }
    /* timeout */
    _dbg_s("[W:"); if(t>0){ for(int i=0;i<t&&i<120;i++){char c=buf[i];_dbg(c>=32&&c<127?c:'.');} } _dbg_s("]\r\n");
    return -1;
}
