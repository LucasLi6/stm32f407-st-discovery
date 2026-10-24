/*
 * Preset model — see preset_store.h.
 */

#include "preset_store.h"
#include "flash_config.h"
#include "servo_motion.h"
#include "hardware/pushrod.h"
#include "hardware/cprod.h"
#include <string.h>

#define PRESET_MAGIC      0x50525354UL   /* "PRST" */
#define PRESET_FLASH_BASE 0x08009880UL

/* on-flash / RAM slot: angle (0..180, 255=OFF) + duration (0..10000 ms) */
typedef struct {
    uint8_t  angle;
    uint8_t  _pad;
    uint16_t dur_ms;
} servo_preset_slot_t;                    /* 4 bytes */

typedef struct {
    uint32_t             magic;
    uint32_t             fw_hash;
    servo_preset_slot_t  servo[PRESET_SERVO_COUNT][PRESET_POSITIONS];  /* 512 */
    uint8_t              pushrod[PRESET_PUSHROD_COUNT][PRESET_POSITIONS]; /* 2*8 = 16 */
    uint8_t              _reserved[40];                               /* pad to 576 */
} preset_flash_t;

static preset_flash_t _p;

static void _load_defaults(void)
{
    rt_memset(&_p, 0, sizeof(_p));

    /* 基础默认：所有舵机预设 = 90° 中心、瞬时；所有推杆预设 = OFF */
    for (int n = 0; n < PRESET_SERVO_COUNT; n++)
        for (int m = 0; m < PRESET_POSITIONS; m++)
        {
            _p.servo[n][m].angle  = PRESET_DEFAULT_ANGLE;
            _p.servo[n][m].dur_ms = 0;
        }
    for (int r = 0; r < PRESET_PUSHROD_COUNT; r++)
        for (int m = 0; m < PRESET_POSITIONS; m++)
            _p.pushrod[r][m] = PRESET_PUSHROD_OFF;

    /* 覆盖：应用 preset_config.h 里用 set / rod 写的默认值（哨兵 n/p = 0xFF） */
    for (int i = 0; i < (int)(sizeof(servo_def_entries) / sizeof(servo_def_entries[0])); i++)
    {
        const servo_def_entry_t *e = &servo_def_entries[i];
        if (e->n == 0xFF) break;
        _p.servo[e->n][e->p].angle  = (uint8_t)e->deg;
        _p.servo[e->n][e->p].dur_ms = e->ms;
    }
    for (int i = 0; i < (int)(sizeof(rod_def_entries) / sizeof(rod_def_entries[0])); i++)
    {
        const rod_def_entry_t *e = &rod_def_entries[i];
        if (e->p == 0xFF) break;
        _p.pushrod[0][e->p] = e->state;
    }
    for (int i = 0; i < (int)(sizeof(cp_def_entries) / sizeof(cp_def_entries[0])); i++)
    {
        const rod_def_entry_t *e = &cp_def_entries[i];
        if (e->p == 0xFF) break;
        _p.pushrod[1][e->p] = e->state;
    }
}

static int _preset_wb(void)
{
    if (_p.magic != PRESET_MAGIC) return 0;
    flash_config_write_words(PRESET_FLASH_BASE, (const uint32_t *)&_p, sizeof(_p) / 4);
    return sizeof(_p);
}

int preset_store_init(void)
{
    _load_defaults();

    uint32_t fw = flash_config_fw_hash();
    const preset_flash_t *f = (const preset_flash_t *)PRESET_FLASH_BASE;
    int need = 0;

    if (f->magic == PRESET_MAGIC && f->fw_hash == fw)
    {
        rt_memcpy(&_p, f, sizeof(_p));          /* flash valid: keep user edits */
    }
    else
    {
        _p.magic = PRESET_MAGIC;                 /* first boot / fw update: defaults */
        _p.fw_hash = fw;
        need = 1;
    }

    flash_config_register(PRESET_FLASH_BASE, sizeof(_p), _preset_wb);
    return need;
}

int preset_get_servo(uint8_t n, uint8_t pos, uint16_t *deg, uint16_t *ms)
{
    if (n > 15 || pos < 1 || pos > PRESET_POSITIONS) return -1;
    *deg = _p.servo[n][pos - 1].angle;
    *ms  = _p.servo[n][pos - 1].dur_ms;
    return 0;
}

int preset_set_servo(uint8_t n, uint8_t pos, uint16_t deg, uint16_t ms)
{
    if (n > 15 || pos < 1 || pos > PRESET_POSITIONS) return -1;
    if (deg != PRESET_SERVO_OFF && deg > 180) deg = 180;
    if (ms > 10000) ms = 10000;

    _p.servo[n][pos - 1].angle  = (uint8_t)deg;
    _p.servo[n][pos - 1].dur_ms = ms;
    flash_config_commit();
    return 0;
}

int preset_apply_servo(uint8_t n, uint8_t pos)
{
    if (n > 15 || pos < 1 || pos > PRESET_POSITIONS) return -1;

    servo_preset_slot_t *s = &_p.servo[n][pos - 1];
    if (s->angle == PRESET_SERVO_OFF)
    {
        servo_motion_off(n);
        return 0;
    }
    servo_motion_move(n, s->angle, s->dur_ms);
    return s->dur_ms;
}

int preset_apply_pushrod(uint8_t pos)
{
    if (pos < 1 || pos > PRESET_POSITIONS) return -1;

    uint8_t st = _p.pushrod[0][pos - 1];
    if (st == PRESET_PUSHROD_UP)       pushrod_set(PUSHROD_UP);
    else if (st == PRESET_PUSHROD_DN)  pushrod_set(PUSHROD_DOWN);
    else                               pushrod_off();
    return 0;
}

int preset_get_pushrod(uint8_t pos, uint8_t *state)
{
    if (pos < 1 || pos > PRESET_POSITIONS) return -1;
    *state = _p.pushrod[0][pos - 1];
    return 0;
}

int preset_set_pushrod(uint8_t pos, uint8_t state)
{
    if (pos < 1 || pos > PRESET_POSITIONS) return -1;
    _p.pushrod[0][pos - 1] = state;
    flash_config_commit();
    return 0;
}

/* ---- CP 推杆预设（rod 索引 1） ---- */

int preset_apply_cprod(uint8_t pos)
{
    if (pos < 1 || pos > PRESET_POSITIONS) return -1;

    uint8_t st = _p.pushrod[1][pos - 1];
    if (st == PRESET_PUSHROD_UP)       cprod_set(CPROD_UP);
    else if (st == PRESET_PUSHROD_DN)  cprod_set(CPROD_DOWN);
    else                               cprod_off();
    return 0;
}

int preset_get_cprod(uint8_t pos, uint8_t *state)
{
    if (pos < 1 || pos > PRESET_POSITIONS) return -1;
    *state = _p.pushrod[1][pos - 1];
    return 0;
}

int preset_set_cprod(uint8_t pos, uint8_t state)
{
    if (pos < 1 || pos > PRESET_POSITIONS) return -1;
    _p.pushrod[1][pos - 1] = state;
    flash_config_commit();
    return 0;
}
