/*
 * Lift platform (升降台) on solid-state relays 2,3.
 *
 * Each relay switches one motor terminal between GND (relay off) and +V (on):
 *   RL2 = up   relay,  RL3 = down relay
 *   UP   = RL2 ON,  RL3 OFF
 *   DN   = RL2 OFF, RL3 ON
 *   OFF  = RL2 OFF, RL3 OFF   (both motor terminals grounded -> stopped)
 *
 * Limit switches (both active-low, S -> GND, pull-up inputs):
 *   bottom = S2: stops a DN move
 *   top    = S3: stops an UP move (reserved, not yet wired)
 *
 * Timed and "until limit" moves run in a dedicated motion thread so the shell
 * stays responsive, and can be interrupted with lift_stop().  A generation
 * counter lets a stop/off supersede a running move, and a mutex serialises all
 * relay writes.  The run loop polls the limit switch for the travel direction
 * every ~CHUNK ms, so the platform stops as soon as it hits the switch — even
 * if the timed duration has not elapsed.
 */

#include "lift.h"
#include "relay.h"
#include <rtdevice.h>
#include <board.h>

#define LIFT_UP_RELAY         2
#define LIFT_DN_RELAY         3

/* brief dead time before a direction switch (avoid shoot-through on an
 * inductive motor load) */
#define LIFT_SWITCH_DELAY_MS  10

/* limit-poll / interrupt granularity */
#define LIFT_CHUNK_MS         20

#define LIFT_THREAD_STACK     1024
#define LIFT_THREAD_PRIORITY  16    /* lower than shell reader (15) */

/* Limit switch pins.  S1 (MP pushrod) = PC0, so the lift sensors follow on
 * PC1 / PC2.  Change these two lines if your wiring differs. */
#define LIFT_BOTTOM_LIMIT_PIN GET_PIN(C, 1)   /* S2: bottom limit */
#define LIFT_TOP_LIMIT_PIN    GET_PIN(C, 2)   /* S3: top limit (reserved) */

static rt_thread_t _tid = RT_NULL;
static struct rt_semaphore _sem;
static struct rt_mutex     _mtx;
static volatile int _busy = 0;
static volatile int _gen  = 0;   /* bumped on every new command; supersedes running move */
static volatile int _dir  = LIFT_DN;
static volatile uint32_t _ms = 0;
static volatile int _end  = 0;   /* 1 = move until limit; 0 = timed */

/* both relays off: both motor terminals grounded -> stopped */
static void apply_off(void)
{
    relay_set(LIFT_UP_RELAY, 0);
    relay_set(LIFT_DN_RELAY, 0);
}

/* energise the requested direction (passes only through the OFF state).
 * Caller must hold _mtx. */
static void apply_state(int dir)
{
    apply_off();
    rt_thread_mdelay(LIFT_SWITCH_DELAY_MS);
    if (dir == LIFT_UP) {
        relay_set(LIFT_UP_RELAY, 1);
        relay_set(LIFT_DN_RELAY, 0);
    } else {
        relay_set(LIFT_UP_RELAY, 0);
        relay_set(LIFT_DN_RELAY, 1);
    }
}

/* limit switch for the travel direction: DN -> bottom (S2), UP -> top (S3).
 * active-low: pressed (S -> GND) reads PIN_LOW. */
static int limit_triggered(int dir)
{
    rt_base_t pin = (dir == LIFT_UP) ? LIFT_TOP_LIMIT_PIN : LIFT_BOTTOM_LIMIT_PIN;
    return (rt_pin_read(pin) == PIN_LOW);
}

static void lift_thread(void *p)
{
    while (1)
    {
        rt_sem_take(&_sem, RT_WAITING_FOREVER);
        int g = _gen;
        int dir = _dir;
        int end_mode = _end;

        /* energise in the requested direction */
        rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
        apply_state(dir);
        rt_mutex_release(&_mtx);

        /* superseded while starting -> leave state to the superseder */
        if (g != _gen) continue;

        /* run phase: timed duration, or until the direction's limit switch
         * triggers (hard stop — the platform must not move past the switch). */
        uint32_t remain = end_mode ? LIFT_END_MAX_MS : _ms;
        while (remain > 0 && g == _gen)
        {
            if (limit_triggered(dir)) break;
            uint32_t c = remain > LIFT_CHUNK_MS ? LIFT_CHUNK_MS : remain;
            rt_thread_mdelay(c);
            remain -= c;
        }

        /* natural end only: stop (both relays off).  If superseded, the
         * superseder owns the relay state and _busy. */
        if (g == _gen)
        {
            rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
            apply_off();
            rt_mutex_release(&_mtx);
            _busy = 0;
        }
    }
}

int lift_init(void)
{
    if (_tid != RT_NULL) return 0;

    rt_sem_init(&_sem, "liftsem", 0, RT_IPC_FLAG_FIFO);
    rt_mutex_init(&_mtx, "liftmux", RT_IPC_FLAG_FIFO);

    /* limit switch inputs (active low, pull-up) */
    rt_pin_mode(LIFT_BOTTOM_LIMIT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(LIFT_TOP_LIMIT_PIN, PIN_MODE_INPUT_PULLUP);

    /* safe start: both relays off (stopped) */
    relay_set(LIFT_UP_RELAY, 0);
    relay_set(LIFT_DN_RELAY, 0);

    _tid = rt_thread_create("lift", lift_thread, RT_NULL,
                            LIFT_THREAD_STACK, LIFT_THREAD_PRIORITY, 10);
    if (_tid == RT_NULL) return -1;
    rt_thread_startup(_tid);
    return 0;
}

int lift_move(int dir, uint32_t ms)
{
    if (_busy) return -1;
    if (ms > LIFT_MAX_MS) ms = LIFT_MAX_MS;

    _gen++;
    _dir  = dir;
    _ms   = ms;
    _end  = 0;
    _busy = 1;
    rt_sem_release(&_sem);
    return 0;
}

int lift_move_end(int dir)
{
    if (_busy) return -1;

    _gen++;
    _dir  = dir;
    _end  = 1;
    _busy = 1;
    rt_sem_release(&_sem);
    return 0;
}

int lift_busy(void)
{
    return _busy;
}

void lift_stop(void)
{
    lift_off();   /* lift has no separate power relay: stop = both relays off */
}

void lift_off(void)
{
    _gen++;
    _busy = 0;
    _end  = 0;
    rt_mutex_take(&_mtx, RT_WAITING_FOREVER);
    apply_off();
    rt_mutex_release(&_mtx);
}
