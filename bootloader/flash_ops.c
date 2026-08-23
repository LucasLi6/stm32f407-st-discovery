/*
 * STM32F4 internal flash operations (bare-metal).
 *
 * F4 key differences from F1:
 *   - Sector erase (FLASH_CR_SER) instead of page erase (FLASH_CR_PER)
 *   - 32-bit word writes with PSIZE=WORD (FLASH_CR_PSIZE_1)
 *   - Mixed sector sizes (16K/64K/128K)
 */

#include "flash_ops.h"
#include <stm32f4xx.h>

static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123;
        FLASH->KEYR = 0xCDEF89AB;
    }
}

static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static void flash_wait_busy(void)
{
    while (FLASH->SR & FLASH_SR_BSY);
}

/* Erase sectors [first, last] inclusive */
void flash_erase_sectors(int first, int last)
{
    flash_unlock();
    for (int s = first; s <= last; s++) {
        FLASH->CR |= FLASH_CR_SER;
        FLASH->CR &= ~FLASH_CR_SNB_Msk;
        FLASH->CR |= (s << FLASH_CR_SNB_Pos);
        FLASH->CR |= FLASH_CR_STRT;
        flash_wait_busy();
        FLASH->CR &= ~FLASH_CR_SER;
    }
    flash_lock();
}

/* Program a single 32-bit word (no erase — only 1→0 transitions allowed) */
void flash_program_word(uint32_t addr, uint32_t value)
{
    flash_unlock();
    FLASH->CR |= FLASH_CR_PG;
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PSIZE_Msk) | FLASH_CR_PSIZE_1;
    *(volatile uint32_t *)addr = value;
    flash_wait_busy();
    FLASH->CR &= ~FLASH_CR_PG;
    flash_lock();
}

/* Write `count` 32-bit words */
void flash_write_words(uint32_t addr, const uint32_t *data, int count)
{
    flash_unlock();
    FLASH->CR |= FLASH_CR_PG;
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PSIZE_Msk) | FLASH_CR_PSIZE_1;
    for (int i = 0; i < count; i++) {
        *(volatile uint32_t *)(addr + i * 4) = data[i];
        flash_wait_busy();
    }
    FLASH->CR &= ~FLASH_CR_PG;
    flash_lock();
}

/* Write bytes with alignment handling */
void flash_write_bytes(uint32_t addr, const uint8_t *data, int size)
{
    if (size <= 0) return;
    uint32_t a = addr;
    int i = 0;

    /* leading byte(s) to align to word boundary */
    while ((a & 3) && i < size) {
        uint32_t tmp = *(volatile uint32_t *)(a & ~3U);
        int shift = (a & 3) * 8;
        tmp &= ~(0xFFU << shift);
        tmp |= ((uint32_t)data[i]) << shift;
        flash_program_word(a & ~3U, tmp);
        a++; i++;
    }

    /* bulk word writes */
    int words = (size - i) / 4;
    if (words > 0) {
        flash_write_words(a, (const uint32_t *)(data + i), words);
        i += words * 4;
        a += words * 4;
    }

    /* trailing byte(s) */
    while (i < size) {
        uint32_t tmp = *(volatile uint32_t *)(a & ~3U);
        int shift = (a & 3) * 8;
        tmp &= ~(0xFFU << shift);
        tmp |= ((uint32_t)data[i]) << shift;
        flash_program_word(a & ~3U, tmp);
        a++; i++;
    }
}
