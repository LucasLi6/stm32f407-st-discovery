/*
 * Servo timed-motion engine — see servo_motion.h.
 */

#include "servo_motion.h"
#include "hardware/pca9685.h"

#define SERVO_COUNT        16
#define SERVO_TICK_MS      20       /* 50 Hz servo refresh granularity */
#define SERVO_THREAD_STACK 1024
#define SERVO_THREAD_PRIO  17       /* below shell reader (15) */

typedef struct {
    volatile int active;
    int16_t  start_cnt;
    int16_t  target_cnt;
    uint32_t total_ticks;
    uint32_t tick;
    uint16_t cur_cnt;      /* last written PWM count = current position */
} chan_t;

static chan_t            _ch[SERVO_COUNT];
static rt_thread_t       _tid = RT_NULL;
static struct rt_mutex   _mtx;

/* deg -> PWM count (same mapping as pca9685.c) */
static uint16_t deg_to_cnt(uint16_t deg)
{
    if (deg > PCA9685_SERVO_MAX_DEG) deg = PCA9685_SERVO_MAX_DEG;
    int32_t off = PCA9685_SERVO_CENTER
                + ((int32_t)deg - PCA9685_SERVO_NEUTRAL_DEG)
                  * PCA9685_SERVO_SLOPE_N / PCA9685_SERVO_SLOPE_D;
    if (off < 0)    off = 0;
    if (off > 4095) off = 4095;
    return (uint16_t)off;
}

static void servo_motion_thread(void *p)
{
    while (1)
    {
        rt_thread_mdelay(SERVO_TICK_MS);
        rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
        for (int n = 0; n < SERVO_COUNT; n++)
        {
            chan_t *c = &_ch[n];
            if (!c->active) continue;

            c->tick++;
            int16_t cnt;
            if (c->tick >= c->total_ticks)
            {
                cnt = c->target_cnt;
                c->active = 0;
            }
            else
            {
                int32_t d = (int32_t)c->target_cnt - c->start_cnt;
                cnt = (int16_t)(c->start_cnt + d * (int32_t)c->tick
                                              / (int32_t)c->total_ticks);
            }
            c->cur_cnt = cnt;
            pca9685_set_pwm(n, 0, (uint16_t)cnt);
        }
        rt_mutex_release(&_mtx);
    }
}

int servo_motion_init(void)
{
    if (_tid != RT_NULL) return 0;

    rt_mutex_init(&_mtx, "svomtx", RT_IPC_FLAG_FIFO);
    for (int n = 0; n < SERVO_COUNT; n++)
    {
        _ch[n].active = 0;
        _ch[n].cur_cnt = PCA9685_SERVO_CENTER;   /* assume neutral at start */
        _ch[n].start_cnt = 0;
        _ch[n].target_cnt = 0;
        _ch[n].total_ticks = 0;
        _ch[n].tick = 0;
    }

    _tid = rt_thread_create("svom", servo_motion_thread, RT_NULL,
                            SERVO_THREAD_STACK, SERVO_THREAD_PRIO, 10);
    if (_tid == RT_NULL) return -1;
    rt_thread_startup(_tid);
    return 0;
}

void servo_motion_set(uint8_t n, uint16_t deg)
{
    if (n > 15) return;
    rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
    _ch[n].active = 0;
    _ch[n].cur_cnt = deg_to_cnt(deg);
    servo_set_angle(n, deg);
    rt_mutex_release(&_mtx);
}

void servo_motion_off(uint8_t n)
{
    if (n > 15) return;
    rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
    _ch[n].active = 0;
    servo_set_off(n);          /* cur_cnt unchanged: remember last position */
    rt_mutex_release(&_mtx);
}

int servo_motion_move(uint8_t n, uint16_t deg, uint16_t ms)
{
    if (n > 15) return -1;
    if (deg > PCA9685_SERVO_MAX_DEG) deg = PCA9685_SERVO_MAX_DEG;
    if (ms > SERVO_MAX_DUR_MS) ms = SERVO_MAX_DUR_MS;

    rt_mutex_take(&_mtx, RT_WAITING_FOREVER);

    int16_t start  = (int16_t)_ch[n].cur_cnt;   /* current position (even mid-move) */
    int16_t target = (int16_t)deg_to_cnt(deg);

    if (ms == 0 || start == target)
    {
        _ch[n].active = 0;
        _ch[n].cur_cnt = target;
        servo_set_angle(n, deg);
        rt_mutex_release(&_mtx);
        return 0;
    }

    _ch[n].start_cnt = start;
    _ch[n].target_cnt = target;
    _ch[n].total_ticks = ms / SERVO_TICK_MS;
    if (_ch[n].total_ticks == 0) _ch[n].total_ticks = 1;
    _ch[n].tick = 0;
    _ch[n].active = 1;

    rt_mutex_release(&_mtx);
    return 0;
}

int servo_motion_busy(uint8_t n)
{
    if (n > 15) return 0;
    return _ch[n].active ? 1 : 0;
}

void servo_motion_stop_all(void)
{
    rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
    for (int n = 0; n < SERVO_COUNT; n++)
    {
        _ch[n].active = 0;
        servo_set_off(n);
    }
    rt_mutex_release(&_mtx);
}
