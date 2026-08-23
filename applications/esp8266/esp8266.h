/*
 * ESP8266 WiFi module driver (AT command mode)
 * Provides TCP Server for remote telnet shell access
 */

#ifndef __ESP8266_H__
#define __ESP8266_H__

#include <rtthread.h>
#include <rtdevice.h>

int esp8266_init(void);
rt_device_t esp8266_get_device(void);

#endif /* __ESP8266_H__ */
