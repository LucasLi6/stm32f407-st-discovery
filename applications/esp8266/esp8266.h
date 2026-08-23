/*
 * ESP8266 WiFi module driver (AT command mode)
 * Provides TCP Server for remote telnet shell access
 */

#ifndef __ESP8266_H__
#define __ESP8266_H__

#include <rtthread.h>
#include <rtdevice.h>

#define WIFI_MODE_STA  0
#define WIFI_MODE_AP   1

extern int  g_wifi_mode;
extern char g_ap_ssid[33];
extern char g_ap_pass[65];

int esp8266_init(void);
rt_device_t esp8266_get_device(void);

#endif /* __ESP8266_H__ */
