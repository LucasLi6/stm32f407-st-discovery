/*
 * Flash partition layout — shared between bootloader and application.
 *
 * STM32F407VET6  512 KB flash, sector size varies (16K/64K/128K)
 *
 *  ┌─────────────── 0x08000000  Sector 0 (16KB)
 *  │  Bootloader    20 KB
 *  ├─────────────── 0x08005000  Sectors 1~5 + sector 6 partial
 *  │  Application  240 KB
 *  ├─────────────── 0x08041000
 *  │  (free)       252 KB       Sector 6(116KB) + Sector 7
 *  └─────────────── 0x08080000
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
#define BOOTLOADER_SIZE              (20 * 1024)
#define BOOTLOADER_END               (BOOTLOADER_BASE + BOOTLOADER_SIZE)

#define APP_BASE                     ((uint32_t)0x08005000)
#define APP_SIZE                     (240 * 1024)
#define APP_END                      (APP_BASE + APP_SIZE)

/* APP erase: sectors 1~6 (sector 1 partial + 2/3/4/5 full + sector 6 partial) */
#define APP_FIRST_SECTOR             1
#define APP_LAST_SECTOR              6

#define WIFI_CONFIG_BASE             FLASH_SECTOR_7_BASE
#define WIFI_CONFIG_SIZE             (2 * 1024)

/* ---- WiFi credential entry (mirrors wifi_config.c layout) ---- */
#define WIFI_ENTRY_SIZE              128
#define WIFI_ENTRY_COUNT             16
#define WIFI_MAGIC_VALID             0xA5A5A5A5
#define WIFI_SSID_MAX                32
#define WIFI_PASS_MAX                64

typedef struct {
    uint32_t magic;          /* 0x00: WIFI_MAGIC_VALID */
    int32_t  priority;       /* 0x04: 1-based, 0=empty */
    char     ssid[32];       /* 0x08 */
    char     password[64];   /* 0x28 */
    char     reserved[24];   /* 0x68: pad to 128 */
} wifi_entry_t;

/* ---- convenience ---- */
#define APP_VTOR                     APP_BASE

#endif /* __PARTITION_H__ */
