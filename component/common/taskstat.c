#include "component/common/taskstat.h"
#include "component/log/log.h"
#include "osal/period.h"
#include "osal/event.h"

#ifndef CONFIG_TASK_STATS
void task_stats_dump(void) { LOGW("stats", "CONFIG_TASK_STATS is off"); }
#else

static uint32_t cyc_per_us;     /* 每微秒的周期数 */
static uint32_t win_ms;         /* 本次统计窗口长度（毫秒） */
static uint32_t last_dump_ms;   /* 上次触发的时刻 */

/* 周期数 -> 微秒 x10（保留一位小数）；打印时用 %lu.%lu 分成整数和小数 */
static uint32_t us10(uint32_t cyc) { return (uint32_t)(((uint64_t)cyc * 10u) / cyc_per_us); }
static uint32_t rate10(uint32_t cnt) { return (win_ms == 0u) ? 0u : (uint32_t)(((uint64_t)cnt * 10000u) / win_ms); }
#define X10(v)   (unsigned long)((v) / 10u), (unsigned long)((v) % 10u)

/* 在关中断的短临界区里把统计拷出来并清零，打印在临界区外做 */
static void dump_period(struct period_task *t, void *ctx)
{
    (void)ctx;
    struct task_stats s;
    uint32_t key = os_irq_save();
    s = t->st;
    uint32_t runs = t->runs - t->st.base[0];
    uint32_t ov   = t->overrun - t->st.base[1];
    t->st.base[0] = t->runs;
    t->st.base[1] = t->overrun;
    task_stats_reset(&t->st);
    os_irq_restore(key);

    uint32_t stk = os_task_stack_free(t->os_task);
    if (s.n == 0u) {
        LOGI("stats", "%-14s P cfg=%luHz run=0/s ov=%lu (没有执行) stk=%luw", t->name,
             (unsigned long)(OS_TICK_HZ / t->div), (unsigned long)ov, (unsigned long)stk);
        return;
    }
    uint32_t r = rate10(runs);
    uint32_t avg = (uint32_t)(s.exec_sum / s.n), lavg = (uint32_t)(s.lat_sum / s.n);
    uint32_t e0 = us10(avg), e1 = us10(s.exec_min), e2 = us10(s.exec_max);
    uint32_t i1 = (s.int_min == 0xFFFFFFFFu) ? 0u : us10(s.int_min), i2 = us10(s.int_max);
    uint32_t l0 = us10(lavg), l1 = us10(s.lat_max);
    LOGI("stats", "%-14s P cfg=%luHz run=%lu.%lu/s ov=%lu exec(us) avg/min/max=%lu.%lu/%lu.%lu/%lu.%lu "
                  "int(us) min/max=%lu.%lu/%lu.%lu lat(us) avg/max=%lu.%lu/%lu.%lu stk=%luw",
         t->name, (unsigned long)(OS_TICK_HZ / t->div), X10(r), (unsigned long)ov,
         X10(e0), X10(e1), X10(e2), X10(i1), X10(i2), X10(l0), X10(l1), (unsigned long)stk);
}

static void dump_event(struct event_task *t, void *ctx)
{
    (void)ctx;
    struct task_stats s;
    uint32_t key = os_irq_save();
    s = t->st;
    uint32_t post = t->posted   - t->st.base[0];
    uint32_t done = t->handled  - t->st.base[1];
    uint32_t tmo  = t->timeouts - t->st.base[2];
    uint32_t drop = t->dropped  - t->st.base[3];
    uint32_t ovsz = t->oversize - t->st.base[4];
    t->st.base[0] = t->posted;   t->st.base[1] = t->handled;  t->st.base[2] = t->timeouts;
    t->st.base[3] = t->dropped;  t->st.base[4] = t->oversize;
    task_stats_reset(&t->st);
    os_irq_restore(key);

    uint32_t stk = os_task_stack_free(t->os_task);
    uint32_t rp = rate10(post), rd = rate10(done);
    if (s.n == 0u) {
        LOGI("stats", "%-14s E post=%lu.%lu/s done=0/s drop=%lu ovsz=%lu tmo=%lu qmax=%u stk=%luw (没有处理)", t->name,
             X10(rp), (unsigned long)drop, (unsigned long)ovsz, (unsigned long)tmo, s.qmax, (unsigned long)stk);
        return;
    }
    uint32_t avg = (uint32_t)(s.exec_sum / s.n), lavg = (uint32_t)(s.lat_sum / s.n);
    uint32_t e0 = us10(avg), e1 = us10(s.exec_min), e2 = us10(s.exec_max);
    uint32_t i1 = (s.int_min == 0xFFFFFFFFu) ? 0u : us10(s.int_min), i2 = us10(s.int_max);
    uint32_t l0 = us10(lavg), l1 = us10(s.lat_max);
    LOGI("stats", "%-14s E post=%lu.%lu/s done=%lu.%lu/s drop=%lu ovsz=%lu tmo=%lu qmax=%u exec(us) avg/min/max=%lu.%lu/%lu.%lu/%lu.%lu "
                  "int(us) min/max=%lu.%lu/%lu.%lu lat(us) avg/max=%lu.%lu/%lu.%lu stk=%luw",
         t->name, X10(rp), X10(rd), (unsigned long)drop, (unsigned long)ovsz, (unsigned long)tmo, s.qmax,
         X10(e0), X10(e1), X10(e2), X10(i1), X10(i2), X10(l0), X10(l1), (unsigned long)stk);
}

void task_stats_dump(void)
{
    uint32_t now = os_time_ms();
    cyc_per_us = task_stats_cycles_per_us();
    win_ms = now - last_dump_ms;
    last_dump_ms = now;
    LOGI("stats", "---- window %lu ms (cpu %lu MHz) ----", (unsigned long)win_ms, (unsigned long)cyc_per_us);
    period_foreach(dump_period, NULL);
    event_foreach(dump_event, NULL);
}
#endif
