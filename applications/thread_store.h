/*
 * Thread-script model — 16 named threads, variable-length lines.
 *
 * Threads live in sector 2 at 0x08009AC0 (header + directory + text).  A
 * dedicated runner thread executes one script sequentially: a timed servo move
 * blocks the script until the channel is idle again (顺序等待).  `stop` sets a
 * flag the runner checks every step so it can abort promptly.
 */

#ifndef __THREAD_STORE_H__
#define __THREAD_STORE_H__

#include <rtthread.h>
#include "preset_config.h"

/* returns 1 if defaults were (re)loaded and a commit is needed, 0 if flash ok */
int  thread_store_init(void);

const char *thread_store_name(int i);
const char *thread_store_text(int i);        /* current script text */
int  thread_store_index(const char *name);   /* -1 if not found */

int  thread_store_run(int i);                /* start script i; -1 if busy */
void thread_store_stop(void);                /* request stop */
int  thread_store_running(void);             /* 1 if a script is running */

/* ---- line editor (one thread at a time) ---- */
int  thread_store_edit_begin(int i);         /* returns line count */
void thread_store_edit_show(void (*cb)(const char *));  /* print numbered lines */
int  thread_store_edit_insert(int after, int count);
int  thread_store_edit_delete(int line, int count);   /* 1-based line, count>=1 */
int  thread_store_edit_set(int line, const char *content); /* 0 ok, -1 bad line, -2 invalid command */
int  thread_store_edit_quit(void);           /* drop blank lines + save + commit */

#endif /* __THREAD_STORE_H__ */
