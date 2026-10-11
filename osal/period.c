#include "osal/period.h"

extern struct period_task *const __start_period_tasks[];
extern struct period_task *const __stop_period_tasks[];

static volatile uint32_t period_started;

void period_entry(void *arg)
{
    struct period_task *t = arg;
    for (;;) {
        os_period_wait(t);
        uint32_t t0 = task_stats_begin(&t->st, t->st.release);   /* 记唤醒延迟、启动间隔 */
        t->fn(t->arg);
        task_stats_end(&t->st, t0);                              /* 记执行时间 */
        t->runs++;
    }
}

int32_t period_start_all(void)
{
    int32_t ret = ERR_OK;
    for (struct period_task *const *p = __start_period_tasks; p < __stop_period_tasks; p++) {
        if (os_period_start(*p) != 0) { ret = ERR_NOMEM; }     /* 创建失败不静默 */
    }
    period_started = 1u;                                        /* 之后 tick 才开始释放：所有任务同一个 tick 起跑 */
    return ret;
}

void period_tick_isr(void)
{
    int32_t woken = 0;
    /* 任务创建好之前、调度器启动之前都不放行：此时 tick 中断已经在跑（HAL 时基），
     * 但没有任务在等信号量，放行没有意义，还可能在调度器未启动时请求上下文切换 */
    if (period_started == 0u || !os_running()) { return; }
    uint32_t stamp = task_stats_now();
    for (struct period_task *const *p = __start_period_tasks; p < __stop_period_tasks; p++) {
        struct period_task *t = *p;
        if (++t->cnt >= t->div) {
            t->cnt = 0;
            if (os_period_release_isr(t, &woken) != 0) { t->overrun++; }
            else                                       { t->st.release = stamp; }
        }
    }
    os_period_yield_isr(woken);
}

void period_foreach(void (*cb)(struct period_task *t, void *ctx), void *ctx)
{
    for (struct period_task *const *p = __start_period_tasks; p < __stop_period_tasks; p++) { cb(*p, ctx); }
}
