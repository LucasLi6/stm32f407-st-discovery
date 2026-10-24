/*
 * Thread-script model — see thread_store.h.
 */

#include "thread_store.h"
#include "flash_config.h"
#include "servo_motion.h"
#include "preset_store.h"
#include "hardware/relay.h"
#include "hardware/pushrod.h"
#include "hardware/cprod.h"
#include "hardware/lift.h"
#include "shell/shell.h"
#include <string.h>
#include <stdlib.h>

#define THREAD_MAGIC      0x54485244UL   /* "THRD" */
#define THREAD_FLASH_BASE 0x08009AC0UL
#define THREAD_FLASH_SIZE 0x2540UL       /* 9536: to end of sector 2 */

typedef struct {
    uint32_t magic;
    uint32_t fw_hash;
    uint32_t used;
    uint32_t reserved;
} thread_flash_hdr_t;                    /* 16 bytes */

typedef struct {
    uint16_t off;
    uint16_t len;
} thread_dir_t;                          /* 4 bytes */

/* ---- RAM model ---- */
static char     _thr_text[THREAD_MAX][THREAD_TEXT_MAX];  /* '\n'-joined lines */
static int      _thr_len[THREAD_MAX];
static uint32_t _magic = 0;
static uint32_t _fw_hash = 0;

/* runner state */
static volatile int       _running = 0;
static volatile int       _stop = 0;
static int                _run_idx = -1;
static rt_thread_t        _runner_tid = RT_NULL;
static struct rt_semaphore _run_sem;

/* nested-thread support: active call stack (recursion check) + per-depth line buffer */
static int  _run_stack[THREAD_MAX];              /* active thread indices */
static int  _run_depth = 0;
static char _run_line[THREAD_MAX][LINE_MAX + 1]; /* one line buffer per nesting depth */

/* editor state */
static int  _edit_idx = -1;
static char _edit[THREAD_MAX_LINES][LINE_MAX];
static int  _edit_n = 0;

/* ---- helpers ---- */

static int _ci_eq(const char *a, const char *b)
{
    while (*a && *b)
    {
        char ca = *a, cb = *b;
        if (ca >= 'a' && ca <= 'z') ca -= 32;
        if (cb >= 'a' && cb <= 'z') cb -= 32;
        if (ca != cb) return 0;
        a++; b++;
    }
    return *a == *b;
}

/* strict non-negative integer parse (mirrors shell's _atoi_strict) */
static int _atoi_strict(const char *s, int *out)
{
    if (!s || !*s) return 0;
    int v = 0;
    for (const char *p = s; *p; p++)
    {
        if (*p < '0' || *p > '9') return 0;
        v = v * 10 + (*p - '0');
        if (v > 100000) return 0;
    }
    *out = v;
    return 1;
}

static int _parse_line(char *line, char **av, int max)
{
    int argc = 0;
    char *p = line;
    while (*p == ' ') p++;
    while (*p && argc < max)
    {
        av[argc++] = p;
        while (*p && *p != ' ') p++;
        if (*p) { *p = 0; p++; while (*p == ' ') p++; }
    }
    return argc;
}

static void _wait_servo(uint8_t n)
{
    while (servo_motion_busy(n) && !_stop) rt_thread_mdelay(5);
}

static int _run_in_stack(int idx)
{
    for (int i = 0; i < _run_depth; i++)
        if (_run_stack[i] == idx) return 1;
    return 0;
}

static void _run_script(int idx);   /* forward decl (defined below) */

/* execute one script line (sequential semantics) */
static void _script_exec_line(char *line)
{
    char *av[8];
    int ac = _parse_line(line, av, 8);
    if (ac == 0) return;

    char *cmd = av[0];
    char c0 = cmd[0];
    if (c0 >= 'a' && c0 <= 'z') c0 -= 32;

    /* S<n> <deg|OFF|P<m>> [ms] */
    if (c0 == 'S' && cmd[1])
    {
        int n = atoi(cmd + 1);
        if (n < 0 || n > 15) return;
        if (ac < 2) return;

        if (_ci_eq(av[1], "OFF")) { servo_motion_off((uint8_t)n); return; }

        if (av[1][0] == 'P' || av[1][0] == 'p')
        {
            int m = atoi(av[1] + 1);
            if (m >= 1 && m <= PRESET_POSITIONS)
            {
                int d = preset_apply_servo((uint8_t)n, (uint8_t)m);
                if (d > 0) _wait_servo((uint8_t)n);
            }
            return;
        }

        int deg = atoi(av[1]);
        int ms  = (ac >= 3) ? atoi(av[2]) : 0;
        servo_motion_move((uint8_t)n, (uint16_t)deg, (uint16_t)ms);
        if (ms > 0) _wait_servo((uint8_t)n);
        return;
    }

    /* RL<n> ON|OFF (relay 1 free; 2,3=LIFT, 4,5=CP, 6-8=MP reserved) */
    if (c0 == 'R' && (cmd[1] == 'L' || cmd[1] == 'l') && cmd[2])
    {
        int n = atoi(cmd + 2);
        if (n != 1) return;
        if (ac < 2) return;
        if (_ci_eq(av[1], "ON"))      relay_set(n, 1);
        else if (_ci_eq(av[1], "OFF")) relay_set(n, 0);
        return;
    }

    /* MP UP|DN|END|OFF|P<m> [ms] —— ms/END 给定时 = 运动并顺序等待 */
    if (c0 == 'M' && (cmd[1] == 'P' || cmd[1] == 'p') && !cmd[2])
    {
        if (ac < 2) return;
        if (_ci_eq(av[1], "OFF")) { pushrod_off(); return; }
        if (_ci_eq(av[1], "END")) { pushrod_move_end(); while (pushrod_busy() && !_stop) rt_thread_mdelay(5); return; }
        if (av[1][0] == 'P' || av[1][0] == 'p')
        {
            int m = atoi(av[1] + 1);
            if (m >= 1 && m <= PRESET_POSITIONS) preset_apply_pushrod((uint8_t)m);
            return;
        }

        int dir;
        if      (_ci_eq(av[1], "UP")) dir = PUSHROD_UP;
        else if (_ci_eq(av[1], "DN")) dir = PUSHROD_DOWN;
        else return;

        if (ac >= 3)
        {
            int ms = atoi(av[2]);
            if (ms < 0) ms = 0;
            if (ms > PUSHROD_MAX_MS) ms = PUSHROD_MAX_MS;
            if (ms > 0)
            {
                pushrod_move(dir, (uint32_t)ms);
                while (pushrod_busy() && !_stop) rt_thread_mdelay(5);
                return;
            }
        }
        pushrod_set(dir);   /* 无 ms = 连续运动（不等待） */
        return;
    }

    /* CP UP|DN|OFF|P<m> [ms] —— 与 MP 同逻辑（继电器 4/5） */
    if (c0 == 'C' && (cmd[1] == 'P' || cmd[1] == 'p') && !cmd[2])
    {
        if (ac < 2) return;
        if (_ci_eq(av[1], "OFF")) { cprod_off(); return; }
        if (av[1][0] == 'P' || av[1][0] == 'p')
        {
            int m = atoi(av[1] + 1);
            if (m >= 1 && m <= PRESET_POSITIONS) preset_apply_cprod((uint8_t)m);
            return;
        }

        int dir;
        if      (_ci_eq(av[1], "UP")) dir = CPROD_UP;
        else if (_ci_eq(av[1], "DN")) dir = CPROD_DOWN;
        else return;

        if (ac >= 3)
        {
            int ms = atoi(av[2]);
            if (ms < 0) ms = 0;
            if (ms > CPROD_MAX_MS) ms = CPROD_MAX_MS;
            if (ms > 0)
            {
                cprod_move(dir, (uint32_t)ms);
                while (cprod_busy() && !_stop) rt_thread_mdelay(5);
                return;
            }
        }
        cprod_set(dir);   /* 无 ms = 连续运动（不等待） */
        return;
    }

    /* LIFT <UP|DN> [ms|end] | LIFT OFF | LIFT STOP —— 升降台（继电器2/3），
     * 定时/到位运动都监控该方向限位，触发即停，脚本顺序等待到结束 */
    if (_ci_eq(cmd, "LIFT"))
    {
        if (ac < 2) return;
        if (_ci_eq(av[1], "OFF") || _ci_eq(av[1], "STOP")) { lift_off(); return; }

        int dir;
        if      (_ci_eq(av[1], "UP")) dir = LIFT_UP;
        else if (_ci_eq(av[1], "DN")) dir = LIFT_DN;
        else return;

        if (ac < 3 || _ci_eq(av[2], "END"))
        {
            lift_move_end(dir);
            while (lift_busy() && !_stop) rt_thread_mdelay(5);
            return;
        }

        int ms;
        if (!_atoi_strict(av[2], &ms) || ms < 0) return;
        if (ms > LIFT_MAX_MS) ms = LIFT_MAX_MS;
        if (ms > 0)
        {
            lift_move(dir, (uint32_t)ms);
            while (lift_busy() && !_stop) rt_thread_mdelay(5);
        }
        return;
    }

    /* Run <线程名> —— 嵌套执行其它线程；目标已在调用栈中（递归）则拒绝 */
    if (c0 == 'R' && _ci_eq(cmd, "RUN"))
    {
        if (ac < 2) return;
        int t = thread_store_index(av[1]);
        if (t < 0) return;                       /* 未知线程名：忽略 */
        if (_run_in_stack(t))
        {
            char msg[THREAD_NAME_MAX + 32];
            rt_snprintf(msg, sizeof(msg), "Recursive Run %s ignored\r\n", thread_store_name(t));
            shell_put(msg);
            return;
        }
        if (_run_depth >= THREAD_MAX) { shell_put("Run nesting too deep\r\n"); return; }
        _run_script(t);
        return;
    }

    /* D <ms> */
    if (c0 == 'D' && !cmd[1])
    {
        if (ac < 2) return;
        int ms = atoi(av[1]);
        if (ms < 0) ms = 0;
        uint32_t remain = (uint32_t)ms;
        while (remain > 0 && !_stop)
        {
            uint32_t c = (remain > 10) ? 10 : remain;
            rt_thread_mdelay(c);
            remain -= c;
        }
        return;
    }
}

/* validate one script line against the command grammar (case-insensitive).
 * Returns 1 if the line is a valid command, 0 otherwise.  Used by the editor
 * so an invalid line can't be saved into a thread. */
static int _validate_line(const char *line)
{
    char buf[LINE_MAX + 1];
    int i = 0;
    while (line[i] && i < LINE_MAX) { buf[i] = line[i]; i++; }
    buf[i] = '\0';

    char *av[8];
    int ac = _parse_line(buf, av, 8);
    if (ac == 0) return 0;   /* blank / whitespace-only: reject */

    char *cmd = av[0];
    char c0 = cmd[0];
    if (c0 >= 'a' && c0 <= 'z') c0 -= 32;

    /* S<n> <deg|OFF|P<m>> [ms] */
    if (c0 == 'S' && cmd[1])
    {
        int n;
        if (!_atoi_strict(cmd + 1, &n) || n < 0 || n > 15) return 0;
        if (ac < 2) return 0;
        if (_ci_eq(av[1], "OFF")) return (ac == 2);
        if (av[1][0] == 'P' || av[1][0] == 'p')
        {
            int m;
            if (!_atoi_strict(av[1] + 1, &m) || m < 1 || m > PRESET_POSITIONS) return 0;
            return (ac == 2);
        }
        int deg;
        if (!_atoi_strict(av[1], &deg) || deg < 0 || deg > 180) return 0;
        if (ac >= 3)
        {
            if (ac > 3) return 0;
            int ms;
            if (!_atoi_strict(av[2], &ms) || ms < 0 || ms > SERVO_MAX_DUR_MS) return 0;
        }
        return 1;
    }

    /* RL<n> ON|OFF (relay 1 free; 2,3=LIFT, 4,5=CP, 6-8=MP reserved) */
    if (c0 == 'R' && (cmd[1] == 'L' || cmd[1] == 'l') && cmd[2])
    {
        int n;
        if (!_atoi_strict(cmd + 2, &n) || n != 1) return 0;
        if (ac != 2) return 0;
        return (_ci_eq(av[1], "ON") || _ci_eq(av[1], "OFF"));
    }

    /* MP / CP UP|DN|OFF|P<m> [ms] */
    int is_mp = (c0 == 'M' && (cmd[1] == 'P' || cmd[1] == 'p') && !cmd[2]);
    int is_cp = (c0 == 'C' && (cmd[1] == 'P' || cmd[1] == 'p') && !cmd[2]);
    if (is_mp || is_cp)
    {
        if (ac < 2) return 0;
        if (_ci_eq(av[1], "OFF")) return (ac == 2);
        if (is_mp && _ci_eq(av[1], "END")) return (ac == 2);   /* mp end: MP only */
        if (_ci_eq(av[1], "UP") || _ci_eq(av[1], "DN"))
        {
            if (ac == 2) return 1;
            if (ac != 3) return 0;
            int ms;
            if (!_atoi_strict(av[2], &ms) || ms < 0 || ms > PUSHROD_MAX_MS) return 0;
            return 1;
        }
        if (av[1][0] == 'P' || av[1][0] == 'p')
        {
            int m;
            if (!_atoi_strict(av[1] + 1, &m) || m < 1 || m > PRESET_POSITIONS) return 0;
            return (ac == 2);
        }
        return 0;
    }

    /* LIFT <UP|DN> [ms|end] | LIFT OFF | LIFT STOP */
    if (_ci_eq(cmd, "LIFT"))
    {
        if (ac < 2) return 0;
        if (_ci_eq(av[1], "OFF") || _ci_eq(av[1], "STOP")) return (ac == 2);
        if (_ci_eq(av[1], "UP") || _ci_eq(av[1], "DN"))
        {
            if (ac == 2) return 1;                        /* 无 ms/end = 到位 */
            if (ac != 3) return 0;
            if (_ci_eq(av[2], "END")) return 1;
            int ms;
            if (!_atoi_strict(av[2], &ms) || ms < 0 || ms > LIFT_MAX_MS) return 0;
            return 1;
        }
        return 0;
    }

    /* Run <线程名> */
    if (c0 == 'R' && _ci_eq(cmd, "RUN"))
    {
        if (ac != 2) return 0;
        return (thread_store_index(av[1]) >= 0);
    }

    /* D <ms> */
    if (c0 == 'D' && !cmd[1])
    {
        if (ac != 2) return 0;
        int ms;
        if (!_atoi_strict(av[1], &ms) || ms < 0) return 0;
        return 1;
    }

    return 0;
}

/* run one thread's script lines.  A "Run <name>" line recurses into that
 * thread (index pushed onto _run_stack so a cycle is refused); the per-depth
 * line buffer keeps each nesting level from clobbering its caller. */
static void _run_script(int idx)
{
    _run_stack[_run_depth++] = idx;
    char *line = _run_line[_run_depth - 1];

    const char *ls = _thr_text[idx];   /* '\0'-terminated */
    while (*ls && !_stop)
    {
        const char *nl = ls;
        while (*nl && *nl != '\n') nl++;
        int len = (int)(nl - ls);
        if (len > LINE_MAX - 1) len = LINE_MAX - 1;
        rt_memcpy(line, ls, len);
        line[len] = '\0';
        _script_exec_line(line);
        ls = (*nl == '\n') ? nl + 1 : nl;
    }

    _run_depth--;
}

static void _runner(void *p)
{
    while (1)
    {
        rt_sem_take(&_run_sem, RT_WAITING_FOREVER);
        int idx = _run_idx;
        _stop = 0;
        _run_depth = 0;

        _run_script(idx);

        _running = 0;
        /* UTF-8 byte escapes (ARMCC V5 reads source as GBK, keep literals ASCII):
         * "<name> 执行完毕" / "<name> 已停止" */
        char msg[THREAD_NAME_MAX + 24];
        rt_snprintf(msg, sizeof(msg), "%s %s\r\n",
                    thread_store_name(idx),
                    _stop ? "\xE5\xB7\xB2\xE5\x81\x9C\xE6\xAD\xA2"
                          : "\xE6\x89\xA7\xE8\xA1\x8C\xE5\xAE\x8C\xE6\xAF\x95");
        shell_put(msg);
    }
}

/* ---- flash writeback ---- */

/* word-aligned text padding scratch — kept OFF the thread stack because
 * flash_config_commit() runs in the small shell reader thread (and main) */
static uint32_t _wb_buf[THREAD_TEXT_MAX / 4 + 1];

static int _thread_wb(void)
{
    if (_magic != THREAD_MAGIC) return 0;

    thread_flash_hdr_t hdr;
    hdr.magic = THREAD_MAGIC;
    hdr.fw_hash = _fw_hash;
    hdr.used = 0;
    hdr.reserved = 0;

    thread_dir_t dir[THREAD_MAX];
    uint32_t cursor = sizeof(thread_flash_hdr_t) + sizeof(dir);   /* 80 */

    for (int i = 0; i < THREAD_MAX; i++)
    {
        if (_thr_len[i] > 0)
        {
            dir[i].off = (uint16_t)cursor;
            dir[i].len = (uint16_t)_thr_len[i];
            /* advance by the padded (word-rounded) size, NOT the raw length:
             * flash_config_write_words() writes whole 32-bit words, so a thread
             * whose raw length is not a multiple of 4 must still consume a
             * word-aligned slot.  Without this, the next thread starts at a
             * non-word-aligned flash address and its 32-bit writes corrupt the
             * preceding/following text. */
            cursor += (_thr_len[i] + 3) & ~3u;
        }
        else { dir[i].off = 0; dir[i].len = 0; }
    }
    hdr.used = cursor;

    flash_config_write_words(THREAD_FLASH_BASE, (const uint32_t *)&hdr, sizeof(hdr) / 4);
    flash_config_write_words(THREAD_FLASH_BASE + sizeof(hdr), (const uint32_t *)dir, sizeof(dir) / 4);

    for (int i = 0; i < THREAD_MAX; i++)
    {
        if (_thr_len[i] > 0)
        {
            int words = (_thr_len[i] + 3) / 4;
            rt_memset(_wb_buf, 0xFF, (uint32_t)words * 4);
            rt_memcpy(_wb_buf, _thr_text[i], _thr_len[i]);
            flash_config_write_words(THREAD_FLASH_BASE + dir[i].off, _wb_buf, words);
        }
    }
    return (int)cursor;
}

/* ---- init ---- */

int thread_store_init(void)
{
    uint32_t fw = flash_config_fw_hash();
    const thread_flash_hdr_t *h = (const thread_flash_hdr_t *)THREAD_FLASH_BASE;
    int need = 0;

    if (h->magic == THREAD_MAGIC && h->fw_hash == fw)
    {
        const thread_dir_t *dir = (const thread_dir_t *)(THREAD_FLASH_BASE + sizeof(thread_flash_hdr_t));
        for (int i = 0; i < THREAD_MAX; i++)
        {
            _thr_len[i] = 0;
            if (dir[i].off && dir[i].len)
            {
                int l = dir[i].len;
                if (l >= THREAD_TEXT_MAX) l = THREAD_TEXT_MAX - 1;
                rt_memcpy(_thr_text[i], (const void *)(THREAD_FLASH_BASE + dir[i].off), l);
                _thr_text[i][l] = '\0';
                _thr_len[i] = l;
            }
            else _thr_text[i][0] = '\0';
        }
        _magic = THREAD_MAGIC;
        _fw_hash = fw;
    }
    else
    {
        for (int i = 0; i < THREAD_MAX; i++)
        {
            const char *s = thread_defs[i].body;
            int l = rt_strlen(s);
            if (l >= THREAD_TEXT_MAX) l = THREAD_TEXT_MAX - 1;
            rt_memcpy(_thr_text[i], s, l);
            _thr_text[i][l] = '\0';
            _thr_len[i] = l;
        }
        _magic = THREAD_MAGIC;
        _fw_hash = fw;
        need = 1;
    }

    flash_config_register(THREAD_FLASH_BASE, THREAD_FLASH_SIZE, _thread_wb);

    rt_sem_init(&_run_sem, "thrsem", 0, RT_IPC_FLAG_FIFO);
    _runner_tid = rt_thread_create("thrun", _runner, RT_NULL, 4096, 18, 10);
    if (_runner_tid != RT_NULL) rt_thread_startup(_runner_tid);

    return need;
}

/* ---- names / index ---- */

const char *thread_store_name(int i)
{
    if (i < 0 || i >= THREAD_MAX) return "";
    return thread_defs[i].name;
}

/* current script text ('\n'-joined, '\0'-terminated; reflects terminal edits) */
const char *thread_store_text(int i)
{
    if (i < 0 || i >= THREAD_MAX) return "";
    return _thr_text[i];
}

int thread_store_index(const char *name)
{
    for (int i = 0; i < THREAD_MAX; i++)
        if (_ci_eq(name, thread_defs[i].name)) return i;
    return -1;
}

/* ---- run / stop ---- */

int thread_store_run(int i)
{
    if (i < 0 || i >= THREAD_MAX) return -1;
    if (_running) return -1;
    _run_idx = i;
    _stop = 0;
    _running = 1;
    rt_sem_release(&_run_sem);
    return 0;
}

void thread_store_stop(void) { _stop = 1; }
int  thread_store_running(void) { return _running; }

/* ---- editor ---- */

static void _copy_line(char *dst, const char *src)
{
    int i = 0;
    while (src[i] && i < LINE_MAX - 1) { dst[i] = src[i]; i++; }
    dst[i] = '\0';
}

int thread_store_edit_begin(int i)
{
    if (i < 0 || i >= THREAD_MAX) return -1;
    _edit_idx = i;
    _edit_n = 0;

    if (_thr_len[i] > 0)
    {
        char tmp[THREAD_TEXT_MAX];
        rt_memcpy(tmp, _thr_text[i], _thr_len[i]);
        tmp[_thr_len[i]] = '\0';

        char *ls = tmp;
        for (int pos = 0; pos <= _thr_len[i] && _edit_n < THREAD_MAX_LINES; pos++)
        {
            if (tmp[pos] == '\n')
            {
                tmp[pos] = '\0';
                _copy_line(_edit[_edit_n++], ls);
                ls = tmp + pos + 1;
            }
        }
        if (_edit_n < THREAD_MAX_LINES && *ls) _copy_line(_edit[_edit_n++], ls);
    }
    return _edit_n;
}

void thread_store_edit_show(void (*cb)(const char *))
{
    char buf[LINE_MAX + 8];
    if (_edit_n == 0) { cb(" 0\r\n"); return; }
    for (int k = 0; k < _edit_n; k++)
    {
        rt_snprintf(buf, sizeof(buf), "%2d  %s\r\n", k + 1, _edit[k]);
        cb(buf);
    }
}

int thread_store_edit_insert(int after, int count)
{
    if (after < 0 || after > _edit_n) return -1;
    if (count < 1) return -1;
    if (count > THREAD_MAX_LINES - _edit_n) return -1;   /* 防 int 溢出 */

    for (int k = _edit_n - 1; k >= after; k--)
        rt_strncpy(_edit[k + count], _edit[k], LINE_MAX - 1);
    for (int k = after; k < after + count; k++)
        _edit[k][0] = '\0';
    _edit_n += count;
    return 0;
}

int thread_store_edit_delete(int line, int count)
{
    if (line < 1 || line > _edit_n) return -1;
    if (count < 1 || count > _edit_n - (line - 1)) return -1;

    int from = line - 1;                       /* 0-based start */
    for (int k = from; k < _edit_n - count; k++)
        rt_strncpy(_edit[k], _edit[k + count], LINE_MAX - 1);
    _edit_n -= count;
    return 0;
}

int thread_store_edit_set(int line, const char *content)
{
    if (line < 1 || line > _edit_n) return -1;
    if (!_validate_line(content)) return -2;   /* 非法命令：拒绝写入 */
    _copy_line(_edit[line - 1], content);
    return 0;
}

int thread_store_edit_quit(void)
{
    if (_edit_idx < 0) return -1;

    char *dst = _thr_text[_edit_idx];
    int len = 0;
    for (int k = 0; k < _edit_n; k++)
    {
        if (_edit[k][0] == '\0') continue;      /* drop blank lines */
        int l = rt_strlen(_edit[k]);
        if (len + l + 1 >= THREAD_TEXT_MAX) break;
        if (len > 0) dst[len++] = '\n';
        rt_memcpy(dst + len, _edit[k], l);
        len += l;
    }
    dst[len] = '\0';
    _thr_len[_edit_idx] = len;
    _edit_idx = -1;

    flash_config_commit();
    return 0;
}
