/*
 * STM32F407VET6 Bootloader — Ymodem (UART1) + TCP/WiFi OTA
 */

#include "partition.h"
#include "flash_ops.h"
#include "ymodem.h"
#include "esp8266_at.h"
#include "ota_config.h"
#include "oled.h"
#include <stm32f4xx.h>
#include <string.h>

extern volatile uint8_t _rb[];
extern volatile int _rb_w, _rb_r;
#define RB_SZ 8192

/* ========================================================================
 * Vector table (Cortex-M4, STM32F407)
 * ======================================================================== */

extern unsigned int Image$$RW_IRAM1$$ZI$$Base;
extern unsigned int Image$$RW_IRAM1$$ZI$$Limit;

void Reset_Handler(void);
void Default_Handler(void);
void SysTick_Handler(void);
void USART2_IRQHandler(void);

#pragma diag_suppress 1296
__attribute__((section("RESET"), used))
const uint32_t _vectors[98] = {
    (uint32_t)0x20005000,
    (uint32_t)Reset_Handler,
    (uint32_t)Default_Handler,      /* NMI       */
    (uint32_t)Default_Handler,      /* HardFault */
    (uint32_t)Default_Handler,      /* MemManage */
    (uint32_t)Default_Handler,      /* BusFault  */
    (uint32_t)Default_Handler,      /* UsageFault*/
    0, 0, 0, 0,
    (uint32_t)Default_Handler,      /* SVCall    */
    (uint32_t)Default_Handler,      /* DebugMon  */
    0,
    (uint32_t)Default_Handler,      /* PendSV    */
    (uint32_t)SysTick_Handler,      /* SysTick   */
    [54] = (uint32_t)USART2_IRQHandler, /* USART2 IRQ=38, index=16+38=54 */
};

void Default_Handler(void) { while (1); }

/* ========================================================================
 * SysTick — 1 ms tick (168 MHz core clock)
 * ======================================================================== */

volatile uint32_t _ym_ms = 0;
void SysTick_Handler(void) { _ym_ms++; }
static void delay_ms(uint32_t ms) {
    uint32_t s = _ym_ms; while ((_ym_ms - s) < ms);
}

/* ========================================================================
 * LED (PA1, active-low)
 * ======================================================================== */

static void led(int on) {
    if (on) GPIOA->BSRR = (1 << 1) << 16;
    else    GPIOA->BSRR = (1 << 1);
}
static void blink(int ms) { led(1); delay_ms(ms); led(0); delay_ms(ms); }
static void uart_puts(const char *s) { while (*s) ymodem_putc(*s++); }

/* ========================================================================
 * Clock — HSE 8MHz → PLL 168MHz
 * ======================================================================== */

static void clock_init(void)
{
    /* Use HSI (internal 16MHz RC) — no external crystal dependency.
     * Try HSE first, fall back to HSI if not ready within ~2ms.
     */
    uint32_t src = RCC_PLLCFGR_PLLSRC_HSI;
    uint32_t pllm = 16;  /* HSI / 16 = 1MHz VCO input */

    RCC->CR |= RCC_CR_HSEON;
    /* Wait up to ~4000 AHB cycles (≈500µs @ HSI 8MHz after reset) */
    for (volatile uint32_t i = 0; i < 4000; i++) {
        if (RCC->CR & RCC_CR_HSERDY) {
            src  = RCC_PLLCFGR_PLLSRC_HSE;
            pllm = 8;   /* HSE / 8 = 1MHz VCO input */
            break;
        }
    }
    /* If HSE didn't start, disable it and use HSI */
    if (src == RCC_PLLCFGR_PLLSRC_HSI)
        RCC->CR &= ~RCC_CR_HSEON;

    /* Flash: 5 wait states, prefetch + data + instruction cache */
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN
               | FLASH_ACR_LATENCY_5WS;

    /* APB prescalers: APB1=/4 (42MHz max), APB2=/2 (84MHz max) */
    RCC->CFGR &= ~(RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    /* PLL: SRC / PLLM * PLLN / PLLP = 1MHz * 336 / 2 = 168MHz
     * PLLQ must match the APP's expected value (7 → 48MHz) or the
     * APP's HAL_RCC_OscConfig will detect a mismatch and fail. */
    RCC->PLLCFGR = (pllm << RCC_PLLCFGR_PLLM_Pos)
                 | (336 << RCC_PLLCFGR_PLLN_Pos)
                 | (0   << RCC_PLLCFGR_PLLP_Pos)   /* 00 = PLLP=2 */
                 | (7   << RCC_PLLCFGR_PLLQ_Pos)   /* PLLQ=7 → 48MHz */
                 | src;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* Switch system clock to PLL */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    /* SysTick: 1ms @ 168MHz */
    SysTick->LOAD = 168000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3 << (1 * 2));
    GPIOA->MODER |=  (1 << (1 * 2));   /* output */
    GPIOA->BSRR = (1 << 1);            /* LED off (active-low) */
}

/* ========================================================================
 * Firmware validation & jump
 * ======================================================================== */

static int firmware_is_valid(void)
{
    uint32_t sp = *(uint32_t *)APP_BASE;
    uint32_t pc = *(uint32_t *)(APP_BASE + 4);
    if (sp < 0x20000000 || sp > 0x20020000) return 0;  /* SRAM 128KB */
    if (pc < APP_BASE   || pc >= APP_END)   return 0;
    return 1;
}

/* Hand the APP a clean clock state before jumping (see note below). */
static void clock_switch_to_hsi(void)
{
    /* 1. ensure HSI (16MHz) is on and stable */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    /* 2. switch SYSCLK back to HSI (SW=00) */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

    /* 3. stop the PLL so the APP can reconfigure it from scratch */
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY);
}

/*
 * Why reset the clock before jumping:
 * The APP's SystemClock_Config() is HSI-only (PLLSRC=HSI, M=16), but this
 * bootloader may have left SYSCLK = PLL-from-HSE (M=8). In that state the
 * APP's HAL_RCC_OscConfig() takes the "SYSCLK==PLL" branch and, seeing a
 * PLLSRC/M mismatch, returns HAL_ERROR -> Error_Handler() dead loop (the
 * symptom: boot says "jump ok" but the APP LED never blinks). Switching back
 * to HSI first makes the APP take the normal "reconfigure PLL" path.
 */
__attribute__((noreturn))
static void jump_to_app(uint32_t addr)
{
    uint32_t sp = *(uint32_t *)addr;
    uint32_t pc = *(uint32_t *)(addr + 4);
    clock_switch_to_hsi();
    SysTick->CTRL = 0;
    __disable_irq();
    SCB->VTOR = addr;
    __set_MSP(sp);
    ((void (*)(void))pc)();
    while (1);
}

/* ========================================================================
 * OTA via Ymodem (UART1)
 * ======================================================================== */

static void ota_ymodem(int can_fallback)
{
    ymodem_uart_init();
    while (1) {
        int fw_size = ymodem_receive(APP_BASE, APP_SIZE);
        if (fw_size > 0) {
            for (int i = 0; i < 3; i++) blink(200);
            jump_to_app(APP_BASE);
        }
        if (can_fallback && firmware_is_valid()) jump_to_app(APP_BASE);
        blink(100); blink(100);
    }
}

/* ========================================================================
 * TCP/WiFi OTA — Ymodem over ESP8266 transparent TCP
 * ======================================================================== */

static char *b_strstr(const char *h, int hl, const char *n) {
    int nl=strlen(n);
    for(int i=0;i<=hl-nl;i++) if(memcmp(h+i,n,nl)==0) return (char*)(h+i);
    return 0;
}

/* ---- TCP Ymodem I/O ---- */
#define PLEN (128+5)

static int tcp_getc(int timeout_ms)
{
    uint32_t start = _ym_ms;
    while (1) {
        int c = -1;
        __disable_irq();
        if (_rb_r != _rb_w) { c = _rb[_rb_r]; _rb_r = (_rb_r + 1) % RB_SZ; }
        __enable_irq();
        if (c >= 0) return c;
        if ((_ym_ms - start) >= (uint32_t)timeout_ms) return -1;
    }
}

static void tcp_putc(int c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = (uint8_t)c;
}

/* TCP Ymodem receiver (128-byte SOH packets, CRC16, EOT handshake) */
static int tcp_ymodem_receive(uint32_t flash_addr, uint32_t max_size)
{
    int offset = 0, first = 1, erased = 0;
    uint8_t buf[1024];

    /* phase 1: send 'C' until sender responds */
    while (1) {
        tcp_putc('C');
        int h = -1;
        { uint32_t ts = _ym_ms;
          while ((_ym_ms - ts) < 1000) {
            int c = tcp_getc(100);
            if (c >= 0 && (c == 0x01 || c == 0x02 || c == 0x04)) { h = c; break; }
          }
        }
        if (h == 0x01 || h == 0x02) {
            int dlen = (h == 0x01) ? 128 : 1024;
            uint8_t raw[PLEN]; raw[0] = (uint8_t)h;
            int ok = 1;
            for (int i = 1; i < dlen + 5; i++) {
                int c = tcp_getc(3000);
                if (c < 0) { ok = 0; break; }
                raw[i] = (uint8_t)c;
            }
            if (!ok || (raw[1] ^ raw[2]) != 0xFF) { tcp_putc(0x15); continue; }
            if (raw[1] == 0) { tcp_putc(0x06); continue; }
            uint16_t calc = crc16(&raw[3], dlen);
            uint16_t recv = ((uint16_t)raw[3+dlen] << 8) | raw[4+dlen];
            if (calc != recv) { tcp_putc(0x15); continue; }
            for (int i = 0; i < dlen; i++) buf[i] = raw[3+i];
            if (offset + dlen <= (int)max_size) {
                if (!erased) {
                    flash_erase_sectors(APP_FIRST_SECTOR, APP_LAST_SECTOR);
                    erased = 1;
                }
                flash_write_bytes(flash_addr + offset, buf, dlen);
                offset += dlen;
            }
            tcp_putc(0x06); first = 0; break;
        }
    }
    if (first) return -1;

    /* phase 2: receive remaining data packets + EOT */
    while (1) {
        int h = -1;
        { uint32_t ts = _ym_ms;
          while ((_ym_ms - ts) < 60000) {
            int c = tcp_getc(100);
            if (c >= 0 && (c == 0x01 || c == 0x02 || c == 0x04)) { h = c; break; }
          }
        }
        if (h < 0) return -1;
        if (h == 0x04) {
            tcp_putc(0x15);  /* NAK */
            { uint32_t ts = _ym_ms; int h2 = -1;
              while ((_ym_ms - ts) < 10000) {
                int c = tcp_getc(100);
                if (c >= 0 && c == 0x04) { h2 = c; break; }
              }
              if (h2 == 0x04) {
                tcp_putc(0x06);  /* ACK */
                tcp_putc('C');
                { uint32_t ts2 = _ym_ms; int h3 = -1;
                  while ((_ym_ms - ts2) < 10000) {
                    int c = tcp_getc(100);
                    if (c >= 0 && (c == 0x01 || c == 0x02)) { h3 = c; break; }
                  }
                  if (h3 == 0x01) {
                    uint8_t raw2[133]; raw2[0] = 0x01; int ok = 1;
                    for (int i = 1; i < 133; i++) {
                        int c = tcp_getc(3000); if (c < 0) { ok = 0; break; }
                        raw2[i] = (uint8_t)c;
                    }
                    if (ok && raw2[1] == 0) tcp_putc(0x06);
                  }
                }
                return offset;
              }
            }
            continue;
        }
        if (h == 0x01 || h == 0x02) {
            int dlen = (h == 0x01) ? 128 : 1024;
            uint8_t raw3[PLEN]; raw3[0] = (uint8_t)h;
            int ok = 1;
            for (int i = 1; i < dlen + 5; i++) {
                int c = tcp_getc(3000);
                if (c < 0) { ok = 0; break; }
                raw3[i] = (uint8_t)c;
            }
            if (!ok || (raw3[1] ^ raw3[2]) != 0xFF) { tcp_putc(0x15); continue; }
            if (raw3[1] == 0) { tcp_putc(0x06); continue; }
            uint16_t calc = crc16(&raw3[3], dlen);
            uint16_t recv = ((uint16_t)raw3[3+dlen] << 8) | raw3[4+dlen];
            if (calc != recv) { tcp_putc(0x15); continue; }
            for (int i = 0; i < dlen; i++) buf[i] = raw3[3+i];
            if (offset + dlen <= (int)max_size) {
                if (!erased) {
                    flash_erase_sectors(APP_FIRST_SECTOR, APP_LAST_SECTOR);
                    erased = 1;
                }
                flash_write_bytes(flash_addr + offset, buf, dlen);
                offset += dlen;
            }
            tcp_putc(0x06);
        }
    }
}

static int tcp_ota(void)
{
    char pc_ip[16] = {0}; int offset = 0;
    esp_uart_init();

    /* 1. AT handshake */
    if(at_cmd("AT\r\n","OK",500)!=0) return -1;

    /* 2. query PC IP via CIPSTATUS */
    {
        char buf[256];
        esp_drain();
        esp_send_str("AT+CIPSTATUS\r\n");
        int n=esp_recv(buf,sizeof(buf),2000,100);
        buf[n]=0;
        char *s=b_strstr(buf,n,"+CIPSTATUS:");
        if(s){
            char *t=b_strstr(s,(int)(buf+n-s),"\"TCP\",\"");
            if(t){t+=7; char *e=b_strstr(t,(int)(buf+n-t),"\"");
                if(e){int l=(int)(e-t); if(l>15)l=15; memcpy(pc_ip,t,l); pc_ip[l]=0;}
            }
        }
        if(!pc_ip[0]) return -1;
    }

    /* 3. erase APP flash, cleanup */
    flash_erase_sectors(APP_FIRST_SECTOR, APP_LAST_SECTOR);
    at_cmd("AT+CIPSERVER=0\r\n","OK",1000);
    for(int lid=0;lid<5;lid++){
        char cb[32],*p=cb;
        memcpy(p,"AT+CIPCLOSE=",12);p+=12;
        *p++='0'+lid; memcpy(p,"\r\n",2);p+=2;*p=0;
        at_cmd(cb,"OK",500);
    }

    /* 4. single-connection (required for transparent CIPSEND) */
    if(at_cmd("AT+CIPMUX=0\r\n","OK",1000)!=0) return -1;

    /* 5. transparent mode */
    if(at_cmd("AT+CIPMODE=1\r\n","OK",1000)!=0) return -1;

    /* 6. CIPSTART to PC:8888 */
    { char c[80],*p=c;
      memcpy(p,"AT+CIPSTART=\"TCP\",\"",19);p+=19;
      memcpy(p,pc_ip,strlen(pc_ip));p+=strlen(pc_ip);
      memcpy(p,"\",8888\r\n",8);p+=8;*p=0;
      if(at_cmd_wait(c,"CONNECT",15000)!=0) return -1;
    }

    /* 7. enter transparent, run Ymodem over TCP */
    esp_send_str("AT+CIPSEND\r\n");
    uart_irq_on();
    delay_ms(300);
    { char d[64]; esp_recv(d,sizeof(d),500,100); }  /* drain prompt */
    for(;;){ int c=-1; __disable_irq(); if(_rb_r!=_rb_w){c=_rb[_rb_r];_rb_r=(_rb_r+1)%RB_SZ;} __enable_irq(); if(c<0&&(USART2->SR&USART_SR_RXNE))c=(int)(USART2->DR&0xFF); if(c<0)break; }

    offset = tcp_ymodem_receive(APP_BASE, APP_SIZE);

    /* exit transparent, restore settings */
    NVIC_DisableIRQ(USART2_IRQn); delay_ms(1000);
    esp_send_str("+++"); delay_ms(1500); NVIC_EnableIRQ(USART2_IRQn);
    at_cmd("AT\r\n","OK",2000);
    at_cmd("AT+CIPCLOSE\r\n","OK",2000);
    at_cmd("AT+CIPMUX=1\r\n","OK",1000);

    /* restart ESP so APP sees a clean module */
    esp_send_str("AT+RST\r\n");
    delay_ms(1000);
    /* strip trailing Ymodem Ctrl-Z (0x1A) padding */
    while(offset>0 && *((uint8_t*)APP_BASE+offset-1)==0x1A) offset--;

    return offset > 0 ? offset : -1;
}

/* ========================================================================
 * Entry point
 * ======================================================================== */

void Reset_Handler(void)
{
    unsigned int *bss = &Image$$RW_IRAM1$$ZI$$Base;
    unsigned int *end = &Image$$RW_IRAM1$$ZI$$Limit;
    while (bss < end) *bss++ = 0;

    /* Init LED GPIO BEFORE clock — proves code runs even if clock hangs */
    gpio_init();

    /* Brief LED pulse: bootloader is alive */
    led(1); for (volatile int i = 0; i < 100000; i++);  /* ~quick flash */
    led(0);

    clock_init();

    /* Init UART early so the OLED probe result can be printed */
    ymodem_uart_init();

    /* ---- boot-check display on OLED ---- */
    OLED_Init();
    OLED_ShowString(1, 1, "BOOT CHECK");

    /* Report which slave address (if any) answered the ACK */
    if      (oled_probe_result == 1) uart_puts("OLED: 0x78 ACK\r\n");
    else if (oled_probe_result == 2) uart_puts("OLED: 0x7A ACK\r\n");
    else                             uart_puts("OLED: NONE\r\n");

    /* Read OTA flag from RTC backup register (survives reset, cleared on power-cycle) */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_DBP;
    uint32_t ota_flag = RTC->BKP0R;
    RTC->BKP0R = 0;  /* clear immediately so reboot doesn't re-enter OTA */

    if (ota_flag == OTA_FLAG_DONE) {
        OLED_ShowString(2, 1, "FW valid, jump");
        if (firmware_is_valid()) jump_to_app(APP_BASE);
    }

    if (ota_flag == OTA_FLAG_SERIAL) {
        OLED_ShowString(2, 1, "OTA: Serial");
        OLED_ShowString(3, 1, "Waiting Ymodem..");
        for (int i = 0; i < 3; i++) blink(100);
        ota_ymodem(1);
    }

    if (ota_flag == OTA_FLAG_WIFI) {
        OLED_ShowString(2, 1, "OTA: WiFi TCP");
        for (int i = 0; i < 3; i++) blink(100);
        int fw = tcp_ota();
        if (fw > 0) {
            OLED_ShowString(2, 1, "OTA done, jump");
            for (int i = 0; i < 3; i++) blink(200);
            jump_to_app(APP_BASE);
        }
        ota_ymodem(1);
    }

    if (firmware_is_valid()) {
        OLED_ShowString(2, 1, "FW OK, start APP");
        for(int i=0;i<2;i++){led(1);delay_ms(100);led(0);delay_ms(100);}
        jump_to_app(APP_BASE);
    }
    OLED_ShowString(2, 1, "No FW, OTA mode");
    for(int i=0;i<3;i++){led(1);delay_ms(100);led(0);delay_ms(100);}
    ota_ymodem(0);
}
