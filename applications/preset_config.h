/*
 * 预设 & 线程脚本 —— 默认值文件（半宏定义 DSL）
 *
 * 在这里用和终端几乎一样的语言写默认值，改完直接编译烧录即可。
 * 固件更新（固件 CRC 变化）时，flash 里由终端做的 set/edit 会被这里的默认值覆盖。
 *
 * ════════════════════════ 语法 ════════════════════════
 *  舵机预设（对应终端 set S<n> P<m> <deg|OFF> [ms]）:
 *      直接在下面 servo_def_entries[] 里一行写一个，格式：
 *          set(s0, p1, 180)          舵机0 预设1 = 180° 瞬时
 *          set(s0, p1, 180, 1000)    舵机0 预设1 = 180°，1000ms 缓转
 *          set(s0, p1, OFF)          舵机0 预设1 = 断电（无 PWM）
 *        其中 s0..s15 = 舵机号；p1..p8 = 预设位；角度 0..180 或 OFF；
 *        ms 可省略（省略 = 瞬时，作为单纯的设定）。未列出的舵机预设默认为 90° 中心、瞬时。
 *
 *  推杆预设（对应终端 set MP P<m> <UP|DN|OFF> / set CP P<m> <UP|DN|OFF>）:
 *      在下面 rod_def_entries[]（MP 推杆）和 cp_def_entries[]（CP 推杆）里各写一个：
 *          rod(p1, UP)               推杆 预设1 = 上（UP / DN / OFF）
 *          cp(p1, UP)                CP 推杆 预设1 = 上（UP / DN / OFF）
 *        未列出的推杆预设默认为 OFF。
 *
 *  线程（对应终端 Run/edit 的线程脚本）:
 *      在下面 thread_defs[] 里一行写一个，脚本用多行字符串拼起来：
 *          THREAD("Thread1",
 *              "s0 p1\n"
 *              "s1 p1\n"
 *              "mp up 6000\n")
 *        脚本就是终端 edit 里的文本，行与行用 \n 分隔。
 *        每行命令：S<n> <deg|OFF|P<m>> [ms] / RL<n> ON|OFF / MP UP|DN|END|OFF [ms] / CP UP|DN|OFF [ms] / LIFT UP|DN [ms|end] / D <ms> / Run <线程名>（可嵌套，禁止递归）
 *
 * ═══════════════ 终端命令速查（大小写不敏感）═══════════════
 *   S<n> <deg|OFF|P<m>> [ms]      舵机 n(0..15)
 *   RL<n> ON|OFF                  继电器 n(1)（2-8 = 执行器）
 *   MP UP|DN|END|OFF|STOP|P<m>    推杆（继电器6/7/8）；END = 下降到限位开关触发
 *   CP UP|DN|OFF|STOP|P<m>        CP 推杆（继电器4/5）
 *   LIFT UP|DN [ms|end]|OFF|STOP  升降台（继电器2/3）；end/省略 = 到该方向限位
 *   set S<n> P<m> <deg|OFF> [ms]  改舵机预设
 *   set MP P<m> UP|DN|OFF         改推杆预设
 *   set CP P<m> UP|DN|OFF         改 CP 推杆预设
 *   show S<n>                     查看舵机 n 的全部预设
 *   show MP                       查看推杆全部预设
 *   show CP                       查看 CP 推杆全部预设
 *   show <线程名>                 查看某线程的全部内部指令
 *   Run <线程名> / edit <线程名> / stop
 */

#ifndef __PRESET_CONFIG_H__
#define __PRESET_CONFIG_H__

#include <rtthread.h>

/* ---- 容量 ---- */
#define PRESET_SERVO_COUNT    16      /* 舵机 0..15 */
#define PRESET_PUSHROD_COUNT  2       /* 推杆：0=MP（继电器6/7/8），1=CP（继电器4/5） */
#define PRESET_POSITIONS      8       /* 每设备 p1..p8 */

#define THREAD_MAX            16      /* 线程数量 */
#define THREAD_MAX_LINES      64      /* 每线程最大行数 */
#define LINE_MAX              64      /* 单行最大字符数 */
#define THREAD_NAME_MAX       16      /* 线程名最大长度 */
#define THREAD_TEXT_MAX       576     /* 每线程文本最大字节（受 flash 总量约束） */

/* ---- 取值常量 ---- */
#define PRESET_SERVO_OFF      255     /* 舵机预设：OFF */
#define PRESET_PUSHROD_UP     0
#define PRESET_PUSHROD_DN     1
#define PRESET_PUSHROD_OFF    2

/* 未 set 的舵机预设的默认角度（90 = 中心） */
#define PRESET_DEFAULT_ANGLE  90

/* ════════════════════════════════════════════════════════════════
 *  ▍DSL 简写宏（供下方用户配置区使用，文件末尾统一 #undef）
 * ════════════════════════════════════════════════════════════════ */
#define s0   0
#define s1   1
#define s2   2
#define s3   3
#define s4   4
#define s5   5
#define s6   6
#define s7   7
#define s8   8
#define s9   9
#define s10 10
#define s11 11
#define s12 12
#define s13 13
#define s14 14
#define s15 15

#define p1 0
#define p2 1
#define p3 2
#define p4 3
#define p5 4
#define p6 5
#define p7 6
#define p8 7

#define OFF PRESET_SERVO_OFF

/* 推杆状态通过 token 拼接，不占用全局宏 */
#define ROD_UP   PRESET_PUSHROD_UP
#define ROD_DN   PRESET_PUSHROD_DN
#define ROD_OFF  PRESET_PUSHROD_OFF

#define CP_UP    PRESET_PUSHROD_UP
#define CP_DN    PRESET_PUSHROD_DN
#define CP_OFF   PRESET_PUSHROD_OFF

/* set(舵机, 位置, 角度|OFF [, 时长ms]) —— ms 可省略（末尾补 0 = 瞬时） */
#define set(...) set_impl(__VA_ARGS__, 0)
#define set_impl(n, p, deg, ms, ...) { n, p, (uint16_t)deg, (uint16_t)ms },

/* rod(位置, UP|DN|OFF) —— MP 推杆 */
#define rod(p, s) { p, ROD_ ## s },

/* cp(位置, UP|DN|OFF) —— CP 推杆 */
#define cp(p, s) { p, CP_ ## s },

/* THREAD(名字, 脚本字符串) */
#define THREAD(name, body) { name, body }

/* ---- 条目类型 ---- */
typedef struct { uint8_t n; uint8_t p; uint16_t deg; uint16_t ms; } servo_def_entry_t;
typedef struct { uint8_t p; uint8_t state; } rod_def_entry_t;
typedef struct { const char *name; const char *body; } thread_def_t;

/* ════════════════════════════════════════════════════════════════
 *  ▍用户配置区 —— 只改这里
 *  ▍set/rod/THREAD 直接作为数组元素写，一行一个，不用加任何续行符
 * ════════════════════════════════════════════════════════════════ */

/* ---- 舵机预设：set(舵机, 位置, 角度|OFF [, 时长ms]) ---- */
static const servo_def_entry_t servo_def_entries[] = {
    set(s0, p1, 8)        /* 舵机0 预设1 = 10° */
    set(s0, p2, 150)       /* 舵机0 预设2 = 170° */
    set(s0, p3, 174)       /* 舵机0 预设3 = 174° */
    set(s1, p1, 75)        /* 舵机1 预设1 = 60° */
    set(s1, p2, 110)       /* 舵机1 预设2 = 100° */
    set(s1, p3, 60)
    
    set(s2, p1, 45, 750)   /* 舵机2 预设1 = 45°，750ms 缓转 */
    set(s2, p2, 140, 750)  /* 舵机2 预设2 = 135°，750ms 缓转 */
    set(s2, p3, 45)   /* 舵机2 预设1 = 45° */
    set(s2, p4, 140)  /* 舵机2 预设2 = 135°*/
    set(s3, p1, 20, 750)   /* 舵机3 预设1 = 25°，750ms 缓转 */
    set(s3, p2, 125, 750)  /* 舵机3 预设2 = 175°，750ms 缓转 */
    set(s3, p3, 20)   /* 舵机3 预设1 = 25° */
    set(s3, p4, 125)  /* 舵机3 预设2 = 175° */
    { 0xFF, 0xFF, 0, 0 }   /* 结束标记，勿删 */
};

/* ---- 推杆预设：rod(位置, UP|DN|OFF) ---- */
static const rod_def_entry_t rod_def_entries[] = {
    rod(p1, UP)            /* MP 推杆 预设1 = 上 */
    rod(p2, DN)            /* MP 推杆 预设2 = 下 */
    { 0xFF, 0xFF }         /* 结束标记，勿删 */
};

/* ---- CP 推杆预设：cp(位置, UP|DN|OFF) ---- */
static const rod_def_entry_t cp_def_entries[] = {
    cp(p1, UP)             /* CP 推杆 预设1 = 上 */
    cp(p2, DN)             /* CP 推杆 预设2 = 下 */
    { 0xFF, 0xFF }         /* 结束标记，勿删 */
};

/* ---- 线程：THREAD(名字, "脚本")，正好 THREAD_MAX 个（勿增删行数） ---- */
static const thread_def_t thread_defs[] = {
    THREAD("main",
        "mp end 2000\n"
        "run bott\n"
        "s0 p1\n"
        "s1 p1\n"
        "mp up 7000\n"
        "s0 p2\n"
        "D 1000\n"
        "mp dn 4000\n"
        "s0 p2\n"
        "mp up 4000\n"
        "run cap\n"
        "mp dn\n"
        "D 2000\n"
        "s0 p3 2000\n"
        "mp end\n"
        ),
    THREAD("bott",
        "s2 p1\n"
        "s2 p2\n"),
    THREAD("cap",
        "s3 p1\n"
        "D 500\n"
        "s3 p2\n"),
    THREAD("Thread4",  ""),
    THREAD("Thread5",  ""),
    THREAD("Thread6",  ""),
    THREAD("Thread7",  ""),
    THREAD("Thread8",  ""),
    THREAD("Thread9",  ""),
    THREAD("Thread10", ""),
    THREAD("Thread11", ""),
    THREAD("Thread12", ""),
    THREAD("Thread13", ""),
    THREAD("Thread14", ""),
    THREAD("Thread15", ""),
    THREAD("Thread16", ""),
};

/* ════════════════════════════════════════════════════════════════
 *  ▍自动生成区 —— 勿手动修改
 * ════════════════════════════════════════════════════════════════ */

/* 编译期校验：线程必须正好 THREAD_MAX 个 */
typedef char _thread_count_must_be_16[
    (sizeof(thread_defs) / sizeof(thread_defs[0]) == THREAD_MAX) ? 1 : -1];

/* ---- 清理短宏，避免污染其它源文件 ---- */
#undef s0
#undef s1
#undef s2
#undef s3
#undef s4
#undef s5
#undef s6
#undef s7
#undef s8
#undef s9
#undef s10
#undef s11
#undef s12
#undef s13
#undef s14
#undef s15
#undef p1
#undef p2
#undef p3
#undef p4
#undef p5
#undef p6
#undef p7
#undef p8
#undef OFF
#undef ROD_UP
#undef ROD_DN
#undef ROD_OFF
#undef CP_UP
#undef CP_DN
#undef CP_OFF
#undef set
#undef set_impl
#undef rod
#undef cp
#undef THREAD

#endif /* __PRESET_CONFIG_H__ */
