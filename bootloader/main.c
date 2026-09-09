/*
 * STM32F407VET6 Bootloader — dual-slot (A/B) OTA
 *
 * Flash layout (see partition.h):
 *   S0           bootloader
 *   S1           boot meta  (slot CRC32 + OTA progress state)
 *   S2           app config (wifi / mode / ota server)
 *   S3~S5  APP1  primary slot, always booted
 *   S6~S7  APP2  byte-level backup, never executed in place
 *
 * OTA over Ymodem (UART1 serial) or Ymodem-over-TCP (ESP8266 WiFi).
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
    uint32_t src = RCC_PLLCFGR_PLLSRC_HSI;
    uint32_t pllm = 16;  /* HSI / 16 = 1MHz VCO input */

    RCC->CR |= RCC_CR_HSEON;
    for (volatile uint32_t i = 0; i < 4000; i++) {
        if (RCC->CR & RCC_CR_HSERDY) {
            src  = RCC_PLLCFGR_PLLSRC_HSE;
            pllm = 8;   /* HSE / 8 = 1MHz VCO input */
            break;
        }
    }
    if (src == RCC_PLLCFGR_PLLSRC_HSI)
        RCC->CR &= ~RCC_CR_HSEON;

    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN
               | FLASH_ACR_LATENCY_5WS;

    RCC->CFGR &= ~(RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    RCC->PLLCFGR = (pllm << RCC_PLLCFGR_PLLM_Pos)
                 | (336 << RCC_PLLCFGR_PLLN_Pos)
                 | (0   << RCC_PLLCFGR_PLLP_Pos)   /* 00 = PLLP=2 */
                 | (7   << RCC_PLLCFGR_PLLQ_Pos)   /* PLLQ=7 → 48MHz */
                 | src;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

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
 * CRC32 (reflected, poly 0xEDB88320) — used for slot integrity + compare
 * ======================================================================== */

static uint32_t crc32_table[256];
static int crc32_ready = 0;

static void crc32_init(void)
{
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int j = 0; j < 8; j++)
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        crc32_table[i] = c;
    }
    crc32_ready = 1;
}

static uint32_t crc32(const uint8_t *data, uint32_t len)
{
    if (!crc32_ready) crc32_init();
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++)
        crc = crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

/* ========================================================================
 * Slot helpers
 * ======================================================================== */

/* An erased (never-written) slot starts with 0xFFFFFFFF. */
static int slot_empty(uint32_t base)
{
    return (*(volatile uint32_t*)base == 0xFFFFFFFFu);
}

/* A bootable firmware has sane SP (in 128KB SRAM) and PC (inside APP1).
 * NOTE: APP2 is a byte-for-byte backup of APP1, so its vector table PC still
 * points into APP1's range (the firmware is linked at APP1_BASE).  Checking
 * against `base` would wrongly reject every valid APP2 backup. */
static int vector_ok(uint32_t base)
{
    uint32_t sp = *(volatile uint32_t*)base;
    uint32_t pc = *(volatile uint32_t*)(base + 4);
    if (sp < 0x20000000u || sp > 0x20020000u) return 0;
    if (pc < APP1_BASE || pc >= APP1_BASE + FW_SIZE)    return 0;
    return 1;
}

/* Erase `dst`'s sectors, then copy FW_SIZE bytes from `src`. */
static void copy_slot(uint32_t dst, uint32_t src)
{
    int first, last;
    if (dst == APP1_BASE) { first = APP1_FIRST_SECTOR; last = APP1_LAST_SECTOR; }
    else                  { first = APP2_FIRST_SECTOR; last = APP2_LAST_SECTOR; }

    static uint32_t buf[256];   /* 1 KB transfer buffer */
    flash_erase_sectors(first, last);
    for (uint32_t off = 0; off < FW_SIZE; off += sizeof(buf)) {
        memcpy(buf, (const void*)(src + off), sizeof(buf));
        flash_write_words(dst + off, buf, sizeof(buf) / 4);
    }
}

/* ========================================================================
 * Boot metadata (S1)
 * ======================================================================== */

static void meta_load(boot_meta_t *m)
{
    const boot_meta_t *p = (const boot_meta_t*)BOOT_META_BASE;
    if (p->magic == BOOT_META_MAGIC) *m = *p;
    else { memset(m, 0, sizeof(*m)); m->magic = BOOT_META_MAGIC; }
}

static void meta_store(const boot_meta_t *m)
{
    flash_erase_sectors(BOOT_META_SECTOR, BOOT_META_SECTOR);
    flash_write_words(BOOT_META_BASE, (const uint32_t*)m, sizeof(boot_meta_t) / 4);
}

/* ========================================================================
 * Jump to APP
 * ======================================================================== */

static void clock_switch_to_hsi(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY);
}

__attribute__((noreturn))
static void jump_to_app(uint32_t addr)
{
    uint32_t sp = *(uint32_t *)addr;
    uint32_t pc = *(uint32_t *)(addr + 4);

    __disable_irq();

    /* WiFi OTA leaves USART2 (ESP8266) in transparent mode with RX IRQ on and
     * the module may still be resetting/spamming RXNE.  Reset the peripheral +
     * clear NVIC so a pending IRQ can't fire during APP startup (which would
     * hang rt_hw_board_init() before the uart driver is ready). */
    NVIC_DisableIRQ(USART2_IRQn);
    NVIC_ClearPendingIRQ(USART2_IRQn);
    RCC->APB1RSTR |=  RCC_APB1RSTR_USART2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;

    clock_switch_to_hsi();
    SysTick->CTRL = 0;

    SCB->VTOR = addr;
    __set_MSP(sp);
    ((void (*)(void))pc)();
    while (1);
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

/* TCP Ymodem receiver.  APP1 must already be erased by the caller.
 * Returns bytes written, or -1 on error / first-packet timeout. */
static int tcp_ymodem_receive(uint32_t flash_addr, uint32_t max_size)
{
    int offset = 0, first = 1;
    uint8_t buf[1024];

    /* phase 1: send 'C' until the sender responds (15s deadline) */
    uint32_t start = _ym_ms;
    while (1) {
        if ((_ym_ms - start) >= OTA_FIRST_PKT_TIMEOUT) return -1;
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
          while ((_ym_ms - ts) < OTA_DATA_TIMEOUT) {
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

    /* 3. close any stale connections (APP1 already erased by caller) */
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

    offset = tcp_ymodem_receive(APP1_BASE, FW_SIZE);

    /* exit transparent, restore settings */
    NVIC_DisableIRQ(USART2_IRQn); delay_ms(1000);
    esp_send_str("+++"); delay_ms(1500); NVIC_EnableIRQ(USART2_IRQn);
    at_cmd("AT\r\n","OK",2000);
    at_cmd("AT+CIPCLOSE\r\n","OK",2000);
    at_cmd("AT+CIPMUX=1\r\n","OK",1000);

    /* restart ESP so APP sees a clean module */
    esp_send_str("AT+RST\r\n");
    delay_ms(1000);

    return offset > 0 ? offset : -1;
}

/* ========================================================================
 * OTA flow (A/B)
 * ======================================================================== */

/* Erase APP1 and record "flashing" in meta, so a power-loss mid-OTA is
 * recoverable on the next boot (restore from APP2). */
static void begin_flash_app1(void)
{
    boot_meta_t meta;
    meta_load(&meta);
    meta.app1_crc = 0;
    meta.state = BOOT_STATE_FLASHING_APP1;
    meta_store(&meta);
    flash_erase_sectors(APP1_FIRST_SECTOR, APP1_LAST_SECTOR);
}

/* Receive one firmware image into APP1.  Returns 1 on a valid image. */
static int receive_firmware(int is_wifi)
{
    int fw;
    if (is_wifi) fw = tcp_ota();
    else { ymodem_uart_init(); fw = ymodem_receive(APP1_BASE, FW_SIZE); }
    return (fw > 0) && vector_ok(APP1_BASE);
}

/* APP1 is valid: copy APP1 -> APP2, persist CRCs, jump. */
static void commit_app1(void)
{
    boot_meta_t meta;
    uint32_t c = crc32((const uint8_t*)APP1_BASE, FW_SIZE);

    OLED_ShowString(2, 1, "Commit A1->A2");

    meta_load(&meta);
    meta.app1_crc = c;
    meta.state = BOOT_STATE_COMMIT_APP2;
    meta_store(&meta);

    copy_slot(APP2_BASE, APP1_BASE);

    meta.app2_crc = c;
    meta.state = BOOT_STATE_NORMAL;
    meta_store(&meta);

    OLED_ShowString(3, 1, "OTA OK");
    for (int i = 0; i < 3; i++) blink(200);
    jump_to_app(APP1_BASE);
}

/* Restore APP1 from APP2 (rollback after a failed OTA), jump. */
static void restore_backup(void)
{
    boot_meta_t meta;
    meta_load(&meta);

    OLED_ShowString(3, 1, "Restore A1<-A2");

    copy_slot(APP1_BASE, APP2_BASE);

    meta.app1_crc = meta.app2_crc;
    meta.state = BOOT_STATE_NORMAL;
    meta_store(&meta);

    jump_to_app(APP1_BASE);
}

/* No valid firmware anywhere: wait forever for a serial Ymodem transfer. */
static void serial_ota_wait(void)
{
    while (1) {
        OLED_ShowString(2, 1, "Wait Ymodem");
        begin_flash_app1();
        int fw = ymodem_receive(APP1_BASE, FW_SIZE);
        if (fw > 0 && vector_ok(APP1_BASE)) {
            commit_app1();   /* copy APP1 -> APP2, then jump */
        }
        OLED_ShowString(3, 1, "Transfer fail");
        blink(100); blink(100);
    }
}

static void ota_flow(int is_wifi)
{
    boot_meta_t meta;
    meta_load(&meta);
    int has_backup = (meta.app2_crc != 0) && vector_ok(APP2_BASE);

    if (is_wifi) OLED_ShowString(1, 1, "OTA WiFi");
    else         OLED_ShowString(1, 1, "OTA Serial");

    begin_flash_app1();

    if (receive_firmware(is_wifi)) {
        commit_app1();   /* copy APP1 -> APP2, then jump */
    }

    OLED_ShowString(2, 1, "OTA FAIL");

    if (has_backup) {
        restore_backup();   /* APP1 <- APP2, then jump */
    }

    /* No backup to roll back to: keep waiting for a valid serial transfer. */
    serial_ota_wait();
}

/* ========================================================================
 * Normal boot: recover interrupted OTA, then enforce the 4 sync rules.
 * Jumps to APP1 if bootable; returns (falls through) when no firmware.
 * ======================================================================== */

static void boot_check_and_run(void)
{
    boot_meta_t meta;
    meta_load(&meta);

    /* ---- recover interrupted OTA (power loss) ---- */
    if (meta.state == BOOT_STATE_FLASHING_APP1) {
        if (meta.app2_crc != 0 && vector_ok(APP2_BASE)) {
            OLED_ShowString(2, 1, "Recover A1<-A2");
            copy_slot(APP1_BASE, APP2_BASE);
            meta.app1_crc = meta.app2_crc;
        }
        meta.state = BOOT_STATE_NORMAL;
        meta_store(&meta);
    } else if (meta.state == BOOT_STATE_COMMIT_APP2) {
        if (vector_ok(APP1_BASE)) {
            uint32_t c = crc32((const uint8_t*)APP1_BASE, FW_SIZE);
            OLED_ShowString(2, 1, "Commit A1->A2");
            copy_slot(APP2_BASE, APP1_BASE);
            meta.app1_crc = c;
            meta.app2_crc = c;
        }
        meta.state = BOOT_STATE_NORMAL;
        meta_store(&meta);
    }

    /* ---- slot states ---- */
    int v1 = !slot_empty(APP1_BASE) && vector_ok(APP1_BASE);
    int v2 = !slot_empty(APP2_BASE) && vector_ok(APP2_BASE);

    /* ---- 4 rules ---- */
    if (!v1 && v2) {
        /* APP1 empty/invalid, APP2 valid: restore */
        OLED_ShowString(2, 1, "A1<-A2 restore");
        copy_slot(APP1_BASE, APP2_BASE);
        meta.app1_crc = meta.app2_crc;
        meta_store(&meta);
        v1 = 1;
    } else if (v1 && !v2) {
        /* APP1 valid, APP2 empty: back it up */
        OLED_ShowString(2, 1, "A2<-A1 backup");
        uint32_t c = crc32((const uint8_t*)APP1_BASE, FW_SIZE);
        copy_slot(APP2_BASE, APP1_BASE);
        meta.app1_crc = c;
        meta.app2_crc = c;
        meta_store(&meta);
    } else if (v1 && v2) {
        uint32_t c1 = crc32((const uint8_t*)APP1_BASE, FW_SIZE);
        if (meta.app1_crc != 0 && meta.app2_crc != 0 && meta.app1_crc == meta.app2_crc) {
            /* stored CRCs agree — just verify APP1 integrity */
            if (c1 != meta.app1_crc) {
                OLED_ShowString(2, 1, "A1 bad restore");
                copy_slot(APP1_BASE, APP2_BASE);
                meta.app1_crc = meta.app2_crc;
                meta_store(&meta);
            }
        } else {
            /* CRCs unknown or disagree — full compare (APP2 authoritative) */
            uint32_t c2 = crc32((const uint8_t*)APP2_BASE, FW_SIZE);
            if (c1 != c2) {
                OLED_ShowString(2, 1, "A1 diff restore");
                copy_slot(APP1_BASE, APP2_BASE);
                meta.app1_crc = c2;
                meta.app2_crc = c2;
                meta_store(&meta);
            } else {
                meta.app1_crc = c1;
                meta.app2_crc = c2;
                meta.state = BOOT_STATE_NORMAL;
                meta_store(&meta);
            }
        }
    }

    /* ---- boot APP1 ---- */
    if (v1 && vector_ok(APP1_BASE)) {
        OLED_ShowString(2, 1, "APP1 start");
        jump_to_app(APP1_BASE);
    }
    /* no firmware: return so the caller can wait for OTA */
}

/* ========================================================================
 * Entry point
 * ======================================================================== */

void Reset_Handler(void)
{
    unsigned int *bss = &Image$$RW_IRAM1$$ZI$$Base;
    unsigned int *end = &Image$$RW_IRAM1$$ZI$$Limit;
    while (bss < end) *bss++ = 0;

    gpio_init();
    led(1); for (volatile int i = 0; i < 100000; i++);  /* ~quick flash */
    led(0);

    clock_init();
    ymodem_uart_init();

    OLED_Init();
    OLED_ShowString(1, 1, "BOOT CHECK");

    if      (oled_probe_result == 1) uart_puts("OLED: 0x78 ACK\r\n");
    else if (oled_probe_result == 2) uart_puts("OLED: 0x7A ACK\r\n");
    else                             uart_puts("OLED: NONE\r\n");

    /* Read OTA flag from RTC backup register (survives reset, cleared on power-cycle) */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_DBP;
    uint32_t ota_flag = RTC->BKP0R;
    RTC->BKP0R = 0;  /* clear immediately so reboot doesn't re-enter OTA */

    if (ota_flag == OTA_FLAG_SERIAL) {
        for (int i = 0; i < 3; i++) blink(100);
        ota_flow(0);
    }

    if (ota_flag == OTA_FLAG_WIFI) {
        for (int i = 0; i < 3; i++) blink(100);
        ota_flow(1);
    }

    /* Normal boot: enforce A/B rules and jump if APP1 is bootable. */
    boot_check_and_run();

    /* No firmware anywhere: wait for the first-ever serial transfer. */
    OLED_ShowString(2, 1, "No FW, OTA mode");
    serial_ota_wait();
}
