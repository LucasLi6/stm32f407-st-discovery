/*
 * STM32F4 internal flash operations (bare-metal, no RTOS dependencies).
 *
 * F4 uses sector erase (FLASH_CR_SER) and 32-bit word writes.
 */

#ifndef __FLASH_OPS_H__
#define __FLASH_OPS_H__

#include "partition.h"

/* Erase flash sectors [first, last] (inclusive). */
void flash_erase_sectors(int first, int last);

/* Program one 32-bit word to flash (1->0 only, no prior erase needed). */
void flash_program_word(uint32_t addr, uint32_t value);

/* Write `count` words (uint32_t) to flash at `addr`. */
void flash_write_words(uint32_t addr, const uint32_t *data, int count);

/* Write `size` bytes to flash at `addr` (handles alignment internally). */
void flash_write_bytes(uint32_t addr, const uint8_t *data, int size);

#endif /* __FLASH_OPS_H__ */
