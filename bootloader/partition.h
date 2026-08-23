/*
 * Flash partition layout — shared between bootloader and application.
 *
 * STM32F407VET6  512 KB flash, sector sizes: 16/16/16/16/64/128/128/128 KB
 *
 *  ┌─────────────── 0x08000000  S0       16 KB   Bootloader code
 *  ├─────────────── 0x08004000  S1       16 KB   Boot meta (slot CRC + OTA state)
 *  ├─────────────── 0x08008000  S2       16 KB   Config (WiFi/mode/OTA-server)
 *  ├─────────────── 0x0800C000  S3~S5   208 KB   APP1  (primary, always booted)
 *  ├─────────────── 0x08040000  S6~S7   256 KB   APP2  (backup copy)
 *  └─────────────── 0x08080000
 *
 * Firmware image is always FW_SIZE (= APP1_SIZE = 208 KB).  APP2 is larger
 * (256 KB) but only its first FW_SIZE bytes are copied / validated, so a
 * restore (APP2 -> APP1) is always safe as long as firmware <= FW_SIZE.
 *
 * Sector boundaries are chosen so every region is a whole number of sectors:
 * no region shares a sector with its neighbour, so each can be erased
 * independently.
 */

#ifndef __PARTITION_H__
#define __PARTITION_H__

#include <stdint.h>

/* ---- chip geometry ---- */
#ifndef FLASH_BASE
#define FLASH_BASE                   0x08000000UL
#endif
#define FLASH_TOTAL_SIZE             (512 * 1024)
#define FLASH_TOTAL_END              (FLASH_BASE + FLASH_TOTAL_SIZE)

/* ---- F4 sector layout ---- */
#define FLASH_SECTOR_0_BASE          ((uint32_t)0x08000000)
#define FLASH_SECTOR_0_SIZE          (16 * 1024)
#define FLASH_SECTOR_1_BASE          ((uint32_t)0x08004000)
#define FLASH_SECTOR_1_SIZE          (16 * 1024)
#define FLASH_SECTOR_2_BASE          ((uint32_t)0x08008000)
#define FLASH_SECTOR_2_SIZE          (16 * 1024)
#define FLASH_SECTOR_3_BASE          ((uint32_t)0x0800C000)
#define FLASH_SECTOR_3_SIZE          (16 * 1024)
#define FLASH_SECTOR_4_BASE          ((uint32_t)0x08010000)
#define FLASH_SECTOR_4_SIZE          (64 * 1024)
#define FLASH_SECTOR_5_BASE          ((uint32_t)0x08020000)
#define FLASH_SECTOR_5_SIZE          (128 * 1024)
#define FLASH_SECTOR_6_BASE          ((uint32_t)0x08040000)
#define FLASH_SECTOR_6_SIZE          (128 * 1024)
#define FLASH_SECTOR_7_BASE          ((uint32_t)0x08060000)
#define FLASH_SECTOR_7_SIZE          (128 * 1024)

/* ---- partitions ---- */
#define BOOTLOADER_BASE              FLASH_BASE
#define BOOTLOADER_SIZE              (16 * 1024)          /* S0 only */

#define BOOT_META_BASE               FLASH_SECTOR_1_BASE  /* S1 */
#define BOOT_META_SIZE               FLASH_SECTOR_1_SIZE
#define BOOT_META_SECTOR             1

#define CONFIG_BASE                  FLASH_SECTOR_2_BASE  /* S2 */
#define CONFIG_SIZE                  FLASH_SECTOR_2_SIZE
#define CONFIG_SECTOR                2

#define APP1_BASE                    FLASH_SECTOR_3_BASE  /* 0x0800C000 */
#define APP1_SIZE                    (208 * 1024)         /* S3+S4+S5 */
#define APP1_END                     (APP1_BASE + APP1_SIZE)
#define APP1_FIRST_SECTOR            3
#define APP1_LAST_SECTOR             5

#define APP2_BASE                    FLASH_SECTOR_6_BASE  /* 0x08040000 */
#define APP2_SIZE                    (256 * 1024)         /* S6+S7 */
#define APP2_FIRST_SECTOR            6
#define APP2_LAST_SECTOR             7

/* Canonical firmware image size == APP1_SIZE.  APP2's extra 48 KB are spare. */
#define FW_SIZE                      APP1_SIZE
#define FW_END                       (APP1_BASE + FW_SIZE)

/* ---- boot metadata (stored at BOOT_META_BASE in S1) ---- */
#define BOOT_META_MAGIC              0x4D455441UL  /* "META" */

/* OTA progress state (persisted so a power-loss mid-OTA is recoverable). */
#define BOOT_STATE_NORMAL            0   /* both slots committed */
#define BOOT_STATE_FLASHING_APP1     1   /* erasing/writing APP1 (uncommitted) */
#define BOOT_STATE_COMMIT_APP2       2   /* APP1 valid, need copy APP1 -> APP2 */

typedef struct {
    uint32_t magic;        /* BOOT_META_MAGIC */
    uint32_t app1_crc;     /* CRC32 over APP1[0..FW_SIZE); 0 = no valid fw */
    uint32_t app2_crc;     /* CRC32 over APP2[0..FW_SIZE); 0 = no valid fw */
    uint32_t state;        /* BOOT_STATE_* */
    uint32_t reserved[4];
} boot_meta_t;

/* ---- convenience ---- */
#define APP_VTOR                     APP1_BASE

#endif /* __PARTITION_H__ */
