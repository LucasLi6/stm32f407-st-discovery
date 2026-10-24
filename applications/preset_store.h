/*
 * Preset model — RAM copy + flash persistence (via flash_config).
 *
 * Presets live in sector 2 at 0x08009880.  Servo presets carry an angle
 * (0..180, or PRESET_SERVO_OFF) and a duration (0..10000 ms); pushrod presets
 * carry UP/DN/OFF.  Per-device, 8 positions each.
 */

#ifndef __PRESET_STORE_H__
#define __PRESET_STORE_H__

#include <rtthread.h>
#include "preset_config.h"

/* returns 1 if defaults were (re)loaded and a commit is needed, 0 if flash ok */
int  preset_store_init(void);

/* apply a preset now (non-blocking).
 * apply_servo returns the preset duration (0 = instant/OFF), -1 on error. */
int  preset_apply_servo(uint8_t n, uint8_t pos);
int  preset_apply_pushrod(uint8_t pos);

/* read a servo preset: *deg 0..180 (or PRESET_SERVO_OFF), *ms 0..10000 */
int  preset_get_servo(uint8_t n, uint8_t pos, uint16_t *deg, uint16_t *ms);

/* write a servo preset and commit to flash */
int  preset_set_servo(uint8_t n, uint8_t pos, uint16_t deg, uint16_t ms);

/* read a pushrod preset: *state = PRESET_PUSHROD_UP/DN/OFF */
int  preset_get_pushrod(uint8_t pos, uint8_t *state);

/* write a pushrod preset (PRESET_PUSHROD_UP/DN/OFF) and commit to flash */
int  preset_set_pushrod(uint8_t pos, uint8_t state);

/* ---- CP 推杆（rod 索引 1）预设，接口同上 ---- */
int  preset_apply_cprod(uint8_t pos);
int  preset_get_cprod(uint8_t pos, uint8_t *state);
int  preset_set_cprod(uint8_t pos, uint8_t state);

#endif /* __PRESET_STORE_H__ */
