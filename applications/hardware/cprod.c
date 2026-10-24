/*
 * Second electric push rod ("CP") on solid-state relays 4,5.
 *
 * 2-relay reversing pair (no separate power switch):
 *   RL4 = direction A, RL5 = direction B.
 *   UP   = RL4 OFF, RL5 OFF
 *   DOWN = RL4 ON,  RL5 ON
 *   OFF  = RL4 ON,  RL5 OFF   (safe / de-energised state)
 *
 * Continuous moves are just a relay state (set directly).  Timed moves run in
 * a dedicated motion thread so the shell stays responsive, and can be
 * interrupted mid-run with cprod_stop().
 *
 * A generation counter lets a continuous set / off / stop supersede a running
 * timed move without the motion thread later re-applying or cutting power over
 * the new state.  A mutex serialises all relay writes.
 */

#include "cprod.h"
#include "relay.h"

#define CPROD_DIR_A_RELAY    4
#define CPROD_DIR_B_RELAY    5

/* brief dead time before a direction switch (SSR switches fast, but a short
 * pause avoids back-EMF / shoot-through on an inductive load) */
#define CPROD_SWITCH_DELAY_MS  10

/* interrupt granularity: the run phase is split into chunks of this size so
 * a stop request is noticed within ~CHUNK ms. */
#define CPROD_CHUNK_MS         20

#define CPROD_THREAD_STACK     1024
#define CPROD_THREAD_PRIORITY  16    /* lower than shell reader (15) */

static rt_thread_t _tid = RT_NULL;
static struct rt_semaphore _sem;
static struct rt_mutex     _mtx;
static volatile int _busy = 0;
static volatile int _gen  = 0;   /* bumped on every new command; supersedes running move */
static volatile int _dir  = CPROD_DOWN;
static volatile uint32_t _ms = 0;

/* safe OFF state: RL4 ON, RL5 OFF */
static void apply_off(void)
{
    relay_set(CPROD_DIR_A_RELAY, 1);
    relay_set(CPROD_DIR_B_RELAY, 0);
}

/* safe direction switch: return to OFF -> set direction (passes only through
 * the OFF state, never an undefined pair).  Caller must hold _mtx. */
static void apply_state(int dir)
{
    apply_off();
    rt_thread_mdelay(CPROD_SWITCH_DELAY_MS);
    if (dir == CPROD_UP) {
        relay_set(CPROD_DIR_A_RELAY, 0);
        relay_set(CPROD_DIR_B_RELAY, 0);
    } else {
        relay_set(CPROD_DIR_A_RELAY, 1);
        relay_set(CPROD_DIR_B_RELAY, 1);
    }
}

static void cprod_thread(void *p)
{
    while (1)
    {
        rt_sem_take(&_sem, RT_WAITING_FOREVER);
        int g = _gen;

        /* energise in the requested direction */
        rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
        apply_state(_dir);
        rt_mutex_release(&_mtx);

        /* superseded while starting -> leave state to the superseder */
        if (g != _gen) continue;

        /* run for the timed duration (interruptible chunks) */
        uint32_t remain = _ms;
        while (remain > 0 && g == _gen)
        {
            uint32_t c = remain > CPROD_CHUNK_MS ? CPROD_CHUNK_MS : remain;
            rt_thread_mdelay(c);
            remain -= c;
        }

        /* natural end only: return to OFF.  If superseded, the superseder owns
         * the relay state and _busy. */
        if (g == _gen)
        {
            rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
            apply_off();
            rt_mutex_release(&_mtx);
            _busy = 0;
        }
    }
}

int cprod_init(void)
{
    if (_tid != RT_NULL) return 0;

    rt_sem_init(&_sem, "cprodsem", 0, RT_IPC_FLAG_FIFO);
    rt_mutex_init(&_mtx, "cprodmux", RT_IPC_FLAG_FIFO);

    _tid = rt_thread_create("cprod", cprod_thread, RT_NULL,
                            CPROD_THREAD_STACK, CPROD_THREAD_PRIORITY, 10);
    if (_tid == RT_NULL) return -1;
    rt_thread_startup(_tid);
    return 0;
}

int cprod_move(int dir, uint32_t ms)
{
    if (_busy) return -1;
    if (ms > CPROD_MAX_MS) ms = CPROD_MAX_MS;

    _gen++;
    _dir  = dir;
    _ms   = ms;
    _busy = 1;
    rt_sem_release(&_sem);
    return 0;
}

int cprod_busy(void)
{
    return _busy;
}

void cprod_set(int dir)
{
    _gen++;
    _busy = 0;
    rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
    apply_state(dir);
    rt_mutex_release(&_mtx);
}

void cprod_off(void)
{
    _gen++;
    _busy = 0;
    rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
    apply_off();
    rt_mutex_release(&_mtx);
}

void cprod_stop(void)
{
    cprod_off();   /* CP 无独立电源继电器，停止 = 回到 OFF */
}
