/*
 * Electric push rod (linear actuator) driver on solid-state relays 6,7,8.
 * H-bridge: relay8 = power switch, relay6/relay7 = current direction.
 *
 * Continuous moves are just a relay state (set directly).  Timed moves run in
 * a dedicated motion thread so the shell stays responsive, and can be
 * interrupted mid-run with pushrod_stop().
 *
 * A generation counter lets a continuous set / off / stop supersede a running
 * timed move without the motion thread later re-applying or cutting power over
 * the new state.  A mutex serialises all relay writes.
 */

#include "pushrod.h"
#include "relay.h"
#include <rtdevice.h>
#include <board.h>

#define PUSHROD_POWER_RELAY    8
#define PUSHROD_DIR_A_RELAY    6
#define PUSHROD_DIR_B_RELAY    7

/* brief dead time between power/direction transitions (SSR switches fast,
 * but a short pause avoids back-EMF / shoot-through on an inductive load) */
#define PUSHROD_SWITCH_DELAY_MS  10

/* interrupt granularity: the run phase is split into chunks of this size so
 * a stop request is noticed within ~CHUNK ms (plus power already cut). */
#define PUSHROD_CHUNK_MS         20

#define PUSHROD_THREAD_STACK     1024
#define PUSHROD_THREAD_PRIORITY  16    /* lower than shell reader (15) */

/* limit switch (PC0): S terminal -> GND, so the pin is pull-up input and
 * reads PIN_LOW while the switch is pressed. */
#define PUSHROD_LIMIT_PIN        GET_PIN(C, 0)

static rt_thread_t _tid = RT_NULL;
static struct rt_semaphore _sem;
static struct rt_mutex     _mtx;
static volatile int _busy = 0;
static volatile int _gen  = 0;   /* bumped on every new command; supersedes running move */
static volatile int _dir  = PUSHROD_DOWN;
static volatile uint32_t _ms = 0;
static volatile int _limit = 0;  /* 1 = move DOWN until the limit switch triggers */

/* safe direction switch: cut power -> set direction -> re-apply power.
 * Caller must hold _mtx. */
static void apply_state(int dir)
{
    relay_set(PUSHROD_POWER_RELAY, 0);
    rt_thread_mdelay(PUSHROD_SWITCH_DELAY_MS);
    if (dir == PUSHROD_UP) {
        relay_set(PUSHROD_DIR_A_RELAY, 1);
        relay_set(PUSHROD_DIR_B_RELAY, 1);
    } else {
        relay_set(PUSHROD_DIR_A_RELAY, 0);
        relay_set(PUSHROD_DIR_B_RELAY, 0);
    }
    rt_thread_mdelay(PUSHROD_SWITCH_DELAY_MS);
    relay_set(PUSHROD_POWER_RELAY, 1);
}

/* limit switch active-low: pressed (S -> GND) reads PIN_LOW */
static int limit_triggered(void)
{
    return (rt_pin_read(PUSHROD_LIMIT_PIN) == PIN_LOW);
}

static void pushrod_thread(void *p)
{
    while (1)
    {
        rt_sem_take(&_sem, RT_WAITING_FOREVER);
        int g = _gen;
        int limit_mode = _limit;

        /* power up in the requested direction */
        rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
        apply_state(_dir);
        rt_mutex_release(&_mtx);

        /* superseded while powering up -> leave state to the superseder */
        if (g != _gen) continue;

        /* run phase: timed duration, or until the limit switch triggers
         * (interruptible chunks so stop is noticed within ~CHUNK ms) */
        uint32_t remain = limit_mode ? PUSHROD_END_MAX_MS : _ms;
        while (remain > 0 && g == _gen)
        {
            if (limit_mode && limit_triggered()) break;
            uint32_t c = remain > PUSHROD_CHUNK_MS ? PUSHROD_CHUNK_MS : remain;
            rt_thread_mdelay(c);
            remain -= c;
        }

        /* natural end only: cut power.  If superseded, the superseder owns
         * the relay state and _busy. */
        if (g == _gen)
        {
            rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
            relay_set(PUSHROD_POWER_RELAY, 0);
            rt_mutex_release(&_mtx);
            _busy = 0;
        }
    }
}

int pushrod_init(void)
{
    if (_tid != RT_NULL) return 0;

    rt_sem_init(&_sem, "prodsem", 0, RT_IPC_FLAG_FIFO);
    rt_mutex_init(&_mtx, "prodmux", RT_IPC_FLAG_FIFO);

    /* limit switch input (PC0, active low) */
    rt_pin_mode(PUSHROD_LIMIT_PIN, PIN_MODE_INPUT_PULLUP);

    _tid = rt_thread_create("prrod", pushrod_thread, RT_NULL,
                            PUSHROD_THREAD_STACK, PUSHROD_THREAD_PRIORITY, 10);
    if (_tid == RT_NULL) return -1;
    rt_thread_startup(_tid);
    return 0;
}

int pushrod_move(int dir, uint32_t ms)
{
    if (_busy) return -1;
    if (ms > PUSHROD_MAX_MS) ms = PUSHROD_MAX_MS;

    _gen++;
    _dir   = dir;
    _ms    = ms;
    _limit = 0;
    _busy  = 1;
    rt_sem_release(&_sem);
    return 0;
}

int pushrod_move_end(void)
{
    if (_busy) return -1;

    _gen++;
    _dir   = PUSHROD_DOWN;
    _limit = 1;
    _busy  = 1;
    rt_sem_release(&_sem);
    return 0;
}

int pushrod_busy(void)
{
    return _busy;
}

void pushrod_set(int dir)
{
    _gen++;
    _busy  = 0;
    _limit = 0;
    rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
    apply_state(dir);
    rt_mutex_release(&_mtx);
}

void pushrod_stop(void)
{
    _gen++;
    _busy  = 0;
    _limit = 0;
    rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
    relay_set(PUSHROD_POWER_RELAY, 0);   /* cut power immediately */
    rt_mutex_release(&_mtx);
}

void pushrod_off(void)
{
    _gen++;
    _busy  = 0;
    _limit = 0;
    rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
    relay_set(PUSHROD_POWER_RELAY, 0);
    relay_set(PUSHROD_DIR_A_RELAY, 0);
    relay_set(PUSHROD_DIR_B_RELAY, 0);
    rt_mutex_release(&_mtx);
}
