/*
 * WiFi credential manager — stored in STM32 internal flash (last 2 KB)
 *
 * Commands:
 *   wifi save <ssid> <password> [priority]
 *   wifi show
 *   wifi delete <priority>
 */

#ifndef __WIFI_CONFIG_H__
#define __WIFI_CONFIG_H__

#include <rtthread.h>

#define WIFI_MAX_ENTRIES    16
#define WIFI_SSID_MAX       32
#define WIFI_PASS_MAX       64

typedef struct {
    int  priority;
    char ssid[WIFI_SSID_MAX + 1];
    char password[WIFI_PASS_MAX + 1];
} wifi_info_t;

/* ---- public API ---- */

/* Load RAM copies from flash + register regions with flash_config.
 * Must be called once at boot, before any other wifi_* / flash_config_commit. */
int wifi_config_init(void);

/* Load all entries from flash, sorted by priority (ascending).
 * Returns count. */
int wifi_config_load(wifi_info_t *entries, int max_entries);

/* Save an entry.  If priority <= 0, append at the end.
 * If priority collides, shift existing entries down. */
int wifi_config_save(const char *ssid, const char *password, int priority);

/* Delete by priority.  Higher priorities shift up. */
int wifi_config_delete(int priority);

/* Swap priorities of two entries. */
int wifi_config_swap(int prio1, int prio2);

/* Print all entries via the provided callback. */
void wifi_config_show(void (*write_fn)(const char *));

/* Try to connect using stored credentials, highest priority first.
 * Stops on first success.  esp_dev must be an open UART to ESP8266.
 * Returns 0 on success, -1 if all fail. */
int wifi_config_connect(rt_device_t esp_dev);

/* ---- WiFi mode (STA / AP) persistence ---- */

/* Load saved mode (0=STA, 1=AP) + AP credentials. Returns 0 if a saved
 * config exists, -1 if none (caller should use defaults). */
int wifi_mode_load(int *mode, char *ssid, int ssid_sz, char *pass, int pass_sz);

/* Save mode + AP credentials to flash. Returns 0 on success. */
int wifi_mode_save(int mode, const char *ssid, const char *pass);

#endif
