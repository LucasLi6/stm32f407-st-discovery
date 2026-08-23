/*
 * OTA configuration.
 *
 * OTA flags are stored in RTC backup register BKP0R:
 *   - Survives system reset (warm boot)
 *   - Cleared on power-cycle (cold boot → normal start)
 *   - No flash erase needed
 *
 * The APP writes BKP0R then reboots; bootloader reads and clears it.
 */

#ifndef __OTA_CONFIG_H__
#define __OTA_CONFIG_H__

/* ---- OTA trigger flags (RTC->BKP0R) ---- */
#define OTA_FLAG_SERIAL     ((uint32_t)0x4F544101)     /* "OTA\1"  Ymodem    */
#define OTA_FLAG_WIFI       ((uint32_t)0x4F544102)     /* "OTA\2"  TCP/WiFi  */
#define OTA_FLAG_DONE        ((uint32_t)0x4F544103)     /* "OTA\3"  TCP done  */

/* ---- timing (milliseconds) ---- */
#define OTA_FIRST_PKT_TIMEOUT  15000 /* max wait for first Ymodem packet   */
#define OTA_WIFI_TIMEOUT       15000 /* max wait for WiFi connection        */
#define OTA_TCP_TIMEOUT        10000 /* max wait for TCP connect            */
#define OTA_DATA_TIMEOUT       30000 /* max silence before giving up        */
#define OTA_IDLE_MS            200   /* line-idle threshold in esp_recv     */

#endif /* __OTA_CONFIG_H__ */
