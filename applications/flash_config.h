/*
 * Unified sector-2 owner — shared flash commit layer.
 *
 * Sector 2 (0x08008000, 16 KB) holds several regions: WiFi, mode, OTA-server,
 * presets and thread scripts.  Because a sector is one erase unit, ANY write to
 * any region must erase the whole sector and rewrite everyone, or the erase
 * would silently destroy the neighbours.
 *
 * Each region module keeps an authoritative RAM copy and registers a "writeback"
 * callback here.  flash_config_commit() erases sector 2 and replays every
 * registered writeback from RAM, so a commit can never lose data.
 */

#ifndef __FLASH_CONFIG_H__
#define __FLASH_CONFIG_H__

#include <rtthread.h>

#define CONFIG_SECTOR   2

/* write back one region from its module's RAM copy; return bytes written */
typedef int (*flash_wb_fn)(void);

void     flash_config_init(void);
void     flash_config_register(uint32_t base, uint32_t size, flash_wb_fn wb);
void     flash_config_commit(void);           /* erase sector 2 + replay all */
void     flash_config_write_words(uint32_t addr, const uint32_t *data, int count);

/* firmware hash = CRC32 of APP1[0..FW_SIZE); used to detect a firmware update.
 * Cached: the first call computes it, later calls return the cached value. */
uint32_t flash_config_fw_hash(void);

#endif /* __FLASH_CONFIG_H__ */
