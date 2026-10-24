/*
 * Unified sector-2 owner — erase / word-write primitives + commit.
 * See flash_config.h for the rationale.
 */

#include "flash_config.h"
#include <rtdevice.h>
#include <board.h>
#include <stm32f4xx.h>
#include <stdint.h>

/* must match partition.h (bootloader/partition.h) */
#define APP1_BASE      0x0800C000UL
#define FW_SIZE        (208 * 1024)

#define MAX_REGIONS    8

static struct {
    uint32_t    base;
    uint32_t    size;
    flash_wb_fn wb;
} _regions[MAX_REGIONS];
static int _region_count = 0;

void flash_config_init(void)
{
    _region_count = 0;
}

void flash_config_register(uint32_t base, uint32_t size, flash_wb_fn wb)
{
    if (_region_count < MAX_REGIONS)
    {
        _regions[_region_count].base = base;
        _regions[_region_count].size = size;
        _regions[_region_count].wb   = wb;
        _region_count++;
    }
}

/* ---- low-level F4 flash primitives ---- */

static void _unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

static void _lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static void _erase_sector(void)
{
    _unlock();
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR &= ~FLASH_CR_SNB_Msk;
    FLASH->CR |= (CONFIG_SECTOR << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_SER;
    _lock();
}

void flash_config_write_words(uint32_t addr, const uint32_t *data, int count)
{
    if (count <= 0) return;
    _unlock();
    FLASH->CR |= FLASH_CR_PG;
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PSIZE_Msk) | FLASH_CR_PSIZE_1;  /* 32-bit */
    for (int i = 0; i < count; i++) {
        *(volatile uint32_t *)(addr + i * 4) = data[i];
        while (FLASH->SR & FLASH_SR_BSY);
    }
    FLASH->CR &= ~FLASH_CR_PG;
    _lock();
}

void flash_config_commit(void)
{
    _erase_sector();
    for (int i = 0; i < _region_count; i++)
        if (_regions[i].wb)
            _regions[i].wb();
}

/* ---- CRC32 (reflected, poly 0xEDB88320), same as the bootloader ---- */

static uint32_t _crc_table[256];
static int      _crc_ready = 0;

static void _crc_init(void)
{
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int j = 0; j < 8; j++)
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        _crc_table[i] = c;
    }
    _crc_ready = 1;
}

static uint32_t _crc32(const uint8_t *data, uint32_t len)
{
    if (!_crc_ready) _crc_init();
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++)
        crc = _crc_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

static uint32_t _hash = 0;
static int      _hash_valid = 0;

uint32_t flash_config_fw_hash(void)
{
    if (!_hash_valid) {
        _hash = _crc32((const uint8_t *)APP1_BASE, FW_SIZE);
        _hash_valid = 1;
    }
    return _hash;
}
