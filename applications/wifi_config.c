/*
 * WiFi credential manager — flash-backed storage (STM32F4 internal flash)
 *
 * Flash layout: Sector 7 (0x08060000, 128KB)
 *   WIFI_CONFIG: first 2 KB, 16 entries × 128 bytes
 *   OTA_CONFIG:  at 0x08061800
 */

#include "wifi_config.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdlib.h>
#include <board.h>

/* ---- STM32F407VET6 Sector 7 ---- */
#define WIFI_FLASH_BASE     0x08060000UL
#define WIFI_FLASH_SIZE     (2 * 1024)
#define WIFI_FLASH_END      (WIFI_FLASH_BASE + WIFI_FLASH_SIZE)
#define WIFI_FLASH_SECTOR   7
#define ENTRY_SIZE          128
#define ENTRY_COUNT         (WIFI_FLASH_SIZE / ENTRY_SIZE)

#define MAGIC_VALID         0xA5A5A5A5

/* ---- flash entry layout (128 bytes, word-aligned) ---- */
typedef struct {
    uint32_t magic;          /* 0x00: MAGIC_VALID */
    int32_t  priority;       /* 0x04: 1-based, 0=empty */
    char     ssid[32];       /* 0x08 */
    char     password[64];   /* 0x28 */
    char     reserved[24];   /* 0x68: pad to 128 */
} flash_entry_t;

/* ---- shared work buffer (one copy, not per-function) ---- */
static flash_entry_t _raw_buf[ENTRY_COUNT];       /* 2048 B */
static wifi_info_t   _info_buf[WIFI_MAX_ENTRIES];  /* 1632 B */

/* ---- internal flash ops (F4: 32-bit word writes, sector erase) ---- */

static int flash_load_all(flash_entry_t *entries)
{
    const flash_entry_t *src = (const flash_entry_t *)WIFI_FLASH_BASE;
    for (int i = 0; i < ENTRY_COUNT; i++)
        entries[i] = src[i];
    return ENTRY_COUNT;
}

/* F4 sector erase (only sector 7 used here) */
static void flash_erase_config_sector(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }

    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR &= ~FLASH_CR_SNB_Msk;
    FLASH->CR |= (WIFI_FLASH_SECTOR << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_SER;

    FLASH->CR |= FLASH_CR_LOCK;
}

/* F4 32-bit word write */
static void flash_write_words(uint32_t addr, const uint32_t *data, int count)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }

    FLASH->CR |= FLASH_CR_PG;
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PSIZE_Msk) | FLASH_CR_PSIZE_1;  /* 32-bit */
    for (int i = 0; i < count; i++) {
        *(volatile uint32_t *)(addr + i * 4) = data[i];
        while (FLASH->SR & FLASH_SR_BSY);
    }
    FLASH->CR &= ~FLASH_CR_PG;

    FLASH->CR |= FLASH_CR_LOCK;
}

static int flash_save_all(flash_entry_t *entries)
{
    flash_erase_config_sector();

    uint32_t *src = (uint32_t *)entries;
    int count = ENTRY_COUNT * (ENTRY_SIZE / 4);
    int has_data = 0;
    for (int i = 0; i < count; i++) {
        if (src[i] != 0xFFFFFFFF) { has_data = 1; break; }
    }
    if (has_data)
        flash_write_words(WIFI_FLASH_BASE, src, count);

    return 0;
}

static int cmp_priority(const void *a, const void *b)
{
    const wifi_info_t *wa = (const wifi_info_t *)a;
    const wifi_info_t *wb = (const wifi_info_t *)b;
    return wa->priority - wb->priority;
}

/* ---- public API ---- */

int wifi_config_load(wifi_info_t *out, int max_entries)
{
    flash_entry_t *raw = _raw_buf;
    flash_load_all(raw);

    int count = 0;
    for (int i = 0; i < ENTRY_COUNT && count < max_entries; i++)
    {
        if (raw[i].magic == MAGIC_VALID && raw[i].priority > 0)
        {
            out[count].priority = raw[i].priority;
            rt_strncpy(out[count].ssid,     raw[i].ssid,     WIFI_SSID_MAX);
            rt_strncpy(out[count].password, raw[i].password, WIFI_PASS_MAX);
            out[count].ssid[WIFI_SSID_MAX]     = '\0';
            out[count].password[WIFI_PASS_MAX] = '\0';
            count++;
        }
    }

    /* sort by priority ascending */
    if (count > 1)
        qsort(out, count, sizeof(wifi_info_t), cmp_priority);

    return count;
}

int wifi_config_save(const char *ssid, const char *password, int priority)
{
    flash_entry_t *raw = _raw_buf;
    flash_load_all(raw);

    /* determine priority if auto */
    if (priority <= 0)
    {
        int max_p = 0;
        for (int i = 0; i < ENTRY_COUNT; i++)
            if (raw[i].magic == MAGIC_VALID && raw[i].priority > max_p)
                max_p = raw[i].priority;
        priority = max_p + 1;
    }

    /* shift entries with priority >= target down by 1 */
    for (int i = 0; i < ENTRY_COUNT; i++)
    {
        if (raw[i].magic == MAGIC_VALID && raw[i].priority >= priority)
            raw[i].priority++;
    }

    /* find a free slot (or overwrite the one we just shifted) */
    int slot = -1;
    for (int i = 0; i < ENTRY_COUNT; i++)
    {
        if (raw[i].magic != MAGIC_VALID || raw[i].priority == 0)
            { slot = i; break; }
    }
    if (slot < 0) return -1;  /* no free slot */

    /* write new entry */
    rt_memset(&raw[slot], 0, sizeof(raw[slot]));
    raw[slot].magic    = MAGIC_VALID;
    raw[slot].priority = priority;
    rt_strncpy(raw[slot].ssid,     ssid,     32);
    rt_strncpy(raw[slot].password, password, 64);
    raw[slot].ssid[31]     = '\0';
    raw[slot].password[63] = '\0';

    return flash_save_all(raw);
}

int wifi_config_delete(int priority)
{
    flash_entry_t *raw = _raw_buf;
    flash_load_all(raw);

    int found = 0;
    for (int i = 0; i < ENTRY_COUNT; i++)
    {
        if (raw[i].magic == MAGIC_VALID && raw[i].priority == priority)
        {
            /* mark deleted */
            rt_memset(&raw[i], 0, sizeof(raw[i]));
            found = 1;
        }
        else if (raw[i].magic == MAGIC_VALID && raw[i].priority > priority)
        {
            /* shift up */
            raw[i].priority--;
        }
    }

    if (!found) return -1;

    return flash_save_all(raw);
}

int wifi_config_swap(int prio1, int prio2)
{
    flash_entry_t *raw = _raw_buf;
    flash_load_all(raw);

    int idx1 = -1, idx2 = -1;
    for (int i = 0; i < ENTRY_COUNT; i++) {
        if (raw[i].magic == MAGIC_VALID && raw[i].priority == prio1) idx1 = i;
        if (raw[i].magic == MAGIC_VALID && raw[i].priority == prio2) idx2 = i;
    }
    if (idx1 < 0 || idx2 < 0) return -1;

    /* swap priorities */
    int tmp = raw[idx1].priority;
    raw[idx1].priority = raw[idx2].priority;
    raw[idx2].priority = tmp;

    return flash_save_all(raw);
}

void wifi_config_show(void (*write_fn)(const char *))
{
    wifi_info_t *entries = _info_buf;
    int count = wifi_config_load(entries, WIFI_MAX_ENTRIES);

    char buf[128];
    rt_snprintf(buf, sizeof(buf), "\r\n  %-4s %-20s %s\r\n", "Prio", "SSID", "Password");
    write_fn(buf);
    write_fn("  --------------------------------------------------\r\n");

    for (int i = 0; i < count; i++)
    {
        rt_snprintf(buf, sizeof(buf), "  %-4d %-20s %s\r\n",
                    entries[i].priority, entries[i].ssid, entries[i].password);
        write_fn(buf);
    }

    if (count == 0)
        write_fn("  (no saved networks)\r\n");
    write_fn("\r\n");
}

/* ---- AT-command helpers (used by connect) ---- */

static int _esp_read(rt_device_t dev, char *buf, int max, int timeout_ms)
{
    int total = 0, tick = rt_tick_get();
    while (total < max - 1) {
        char ch;
        if (rt_device_read(dev, 0, &ch, 1) == 1)
            { buf[total++] = ch; tick = rt_tick_get(); }
        else if (rt_tick_get() - tick > rt_tick_from_millisecond(timeout_ms))
            break;
    }
    buf[total] = '\0';
    return total;
}

/* ---- connect ---- */

extern char g_connected_ssid[33];

volatile int _wifi_connecting = 0;  /* 1=CWJAP in progress, 0=idle */

int wifi_config_connect(rt_device_t esp_dev)
{
    wifi_info_t *entries = _info_buf;
    int count = wifi_config_load(entries, WIFI_MAX_ENTRIES);
    if (count == 0) return -1;
    _wifi_connecting = 1;

    /* try each in priority order (1 = highest) */
    for (int i = 0; i < count; i++)
    {
        wifi_info_t *w = &entries[i];
        rt_kprintf("[WiFi] Trying [%d] %s ...\r\n", w->priority, w->ssid);

        /* disconnect any previous connection first */
        {
            char dbuf[64];
            rt_kprintf("[WiFi] send: AT+CWQAP\r\n");
            rt_device_write(esp_dev, 0, "AT+CWQAP\r\n", 10);
            _esp_read(esp_dev, dbuf, sizeof(dbuf), 2000);
        }

        char cmd[160];
        rt_snprintf(cmd, sizeof(cmd),
                    "AT+CWJAP=\"%s\",\"%s\"\r\n", w->ssid, w->password);
        /* drain, send, wait for "WIFI GOT IP" (max 10s) */
        {
            char buf[256];
            while (rt_device_read(esp_dev, 0, buf, 1) > 0);
            rt_device_write(esp_dev, 0, cmd, rt_strlen(cmd));
            int len = _esp_read(esp_dev, buf, sizeof(buf), 15000);
            if (len > 0 && rt_strstr(buf, "WIFI GOT IP"))
            {
                rt_kprintf("[WiFi] Connected!  SSID: %s\r\n", w->ssid);
                rt_strncpy(g_connected_ssid, w->ssid, 32);
                g_connected_ssid[32] = '\0';
                _wifi_connecting = 0;
                return 0;
            }
            rt_kprintf("[WiFi] Failed (timeout or wrong password)\r\n");
        }
    }

    rt_kprintf("[WiFi] All networks exhausted.\r\n");
    _wifi_connecting = 0;
    return -1;
}

/* ---- OTA server config (Sector 7, 0x08061800) ---- */

#define OTA_CONFIG_MAGIC   0x4F544143  /* "OTAC" */
#define OTA_CONFIG_BASE    0x08061800
#define OTA_IP_MAX         32
#define OTA_PATH_MAX       64

typedef struct {
    uint32_t magic;
    char     server_ip[OTA_IP_MAX];
    uint16_t server_port;
    char     http_path[OTA_PATH_MAX];
    char     reserved[22];
} ota_config_raw_t;

static ota_config_raw_t _ota_cfg;

void ota_config_load_defaults(void)
{
    rt_memset(&_ota_cfg, 0xFF, sizeof(_ota_cfg));
}

int ota_config_get(char *ip, int ip_size, int *port, char *path, int path_size)
{
    const ota_config_raw_t *cfg = (const ota_config_raw_t *)OTA_CONFIG_BASE;
    if (cfg->magic == OTA_CONFIG_MAGIC && cfg->server_ip[0] != 0xFF) {
        rt_strncpy(ip,   cfg->server_ip,  ip_size - 1);
        *port = cfg->server_port;
        rt_strncpy(path, cfg->http_path,  path_size - 1);
        ip[ip_size - 1]     = '\0';
        path[path_size - 1] = '\0';
        return 0;
    }
    return -1;
}

int ota_config_set(const char *ip, int port, const char *path)
{
    rt_memset(&_ota_cfg, 0, sizeof(_ota_cfg));
    _ota_cfg.magic = OTA_CONFIG_MAGIC;
    rt_strncpy(_ota_cfg.server_ip,  ip,   OTA_IP_MAX - 1);
    _ota_cfg.server_port = (uint16_t)port;
    rt_strncpy(_ota_cfg.http_path,  path, OTA_PATH_MAX - 1);

    /* write OTA config (sector already erased with WiFi config) */
    flash_write_words(OTA_CONFIG_BASE, (uint32_t *)&_ota_cfg,
                      (sizeof(_ota_cfg) + 3) / 4);
    return 0;
}
