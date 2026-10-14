#include "component/common/taskstat.h"
#include "component/log/log.h"
#include "osal/period.h"
#include "osal/event.h"
#ifdef CONFIG_SHELL
#include "component/shell/shell_cmd.h"
#endif

#ifndef CONFIG_TASK_STATS
void task_stats_dump(void)
{
    LOGW("stats", "CONFIG_TASK_STATS is off");
}
#else

static uint32_t cycles_per_us;      /* 每微秒的周期数 */
static uint32_t window_ms;          /* 本次统计窗口长度（毫秒） */
static uint32_t last_dump_ms;       /* 上次触发的时刻 */

/* 周期数 -> 微秒 x10（保留一位小数）；打印时用 %lu.%lu 分成整数和小数 */
static uint32_t cycles_to_us_x10(uint32_t cycles)
{
    return (uint32_t)(((uint64_t)cycles * 10u) / cycles_per_us);
}

static uint32_t rate_per_second_x10(uint32_t count)
{
    return (window_ms == 0u) ? 0u : (uint32_t)(((uint64_t)count * 10000u) / window_ms);
}

#define X10(value)   (unsigned long)((value) / 10u), (unsigned long)((value) % 10u)

/* 在关中断的短临界区里把统计拷出来并清零，打印在临界区外做 */
static void dump_period(struct period_task *task, void *context)
{
    (void)context;
    struct task_stats stats;
    uint32_t irq_key = os_irq_save();

    stats = task->stats;
    uint32_t run_delta = task->runs - task->stats.base[0];
    uint32_t overrun_delta = task->overrun - task->stats.base[1];

    task->stats.base[0] = task->runs;
    task->stats.base[1] = task->overrun;
    task_stats_reset(&task->stats);
    os_irq_restore(irq_key);

    uint32_t stack_free_words = os_task_stack_free(task->os_task);

    if (stats.run_count == 0u)
    {
        LOGI("stats", "%-14s P cfg=%luHz run=0/s ov=%lu (没有执行) stk=%luw", task->name,
             (unsigned long)(OS_TICK_HZ / task->div), (unsigned long)overrun_delta, (unsigned long)stack_free_words);
        return;
    }
    uint32_t run_rate = rate_per_second_x10(run_delta);
    uint32_t exec_avg_cycles = (uint32_t)(stats.exec_sum / stats.run_count);
    uint32_t latency_avg_cycles = (uint32_t)(stats.latency_sum / stats.run_count);
    uint32_t exec_avg_us = cycles_to_us_x10(exec_avg_cycles);
    uint32_t exec_min_us = cycles_to_us_x10(stats.exec_min);
    uint32_t exec_max_us = cycles_to_us_x10(stats.exec_max);
    uint32_t interval_min_us = (stats.interval_min == 0xFFFFFFFFu) ? 0u : cycles_to_us_x10(stats.interval_min);
    uint32_t interval_max_us = cycles_to_us_x10(stats.interval_max);
    uint32_t latency_avg_us = cycles_to_us_x10(latency_avg_cycles);
    uint32_t latency_max_us = cycles_to_us_x10(stats.latency_max);

    LOGI("stats", "%-14s P cfg=%luHz run=%lu.%lu/s ov=%lu exec(us) avg/min/max=%lu.%lu/%lu.%lu/%lu.%lu "
                  "int(us) min/max=%lu.%lu/%lu.%lu lat(us) avg/max=%lu.%lu/%lu.%lu stk=%luw",
         task->name, (unsigned long)(OS_TICK_HZ / task->div), X10(run_rate), (unsigned long)overrun_delta,
         X10(exec_avg_us), X10(exec_min_us), X10(exec_max_us),
         X10(interval_min_us), X10(interval_max_us),
         X10(latency_avg_us), X10(latency_max_us), (unsigned long)stack_free_words);
}

static void dump_event(struct event_task *task, void *context)
{
    (void)context;
    struct task_stats stats;
    uint32_t irq_key = os_irq_save();

    stats = task->stats;
    uint32_t post_delta = task->posted - task->stats.base[0];
    uint32_t done_delta = task->handled - task->stats.base[1];
    uint32_t timeout_delta = task->timeouts - task->stats.base[2];
    uint32_t drop_delta = task->dropped - task->stats.base[3];
    uint32_t oversize_delta = task->oversize - task->stats.base[4];

    task->stats.base[0] = task->posted;
    task->stats.base[1] = task->handled;
    task->stats.base[2] = task->timeouts;
    task->stats.base[3] = task->dropped;
    task->stats.base[4] = task->oversize;
    task_stats_reset(&task->stats);
    os_irq_restore(irq_key);

    uint32_t stack_free_words = os_task_stack_free(task->os_task);
    uint32_t post_rate = rate_per_second_x10(post_delta);
    uint32_t done_rate = rate_per_second_x10(done_delta);

    if (stats.run_count == 0u)
    {
        LOGI("stats", "%-14s E post=%lu.%lu/s done=0/s drop=%lu ovsz=%lu tmo=%lu qmax=%u stk=%luw (没有处理)", task->name,
             X10(post_rate), (unsigned long)drop_delta, (unsigned long)oversize_delta, (unsigned long)timeout_delta,
             stats.qmax, (unsigned long)stack_free_words);
        return;
    }
    uint32_t exec_avg_cycles = (uint32_t)(stats.exec_sum / stats.run_count);
    uint32_t latency_avg_cycles = (uint32_t)(stats.latency_sum / stats.run_count);
    uint32_t exec_avg_us = cycles_to_us_x10(exec_avg_cycles);
    uint32_t exec_min_us = cycles_to_us_x10(stats.exec_min);
    uint32_t exec_max_us = cycles_to_us_x10(stats.exec_max);
    uint32_t interval_min_us = (stats.interval_min == 0xFFFFFFFFu) ? 0u : cycles_to_us_x10(stats.interval_min);
    uint32_t interval_max_us = cycles_to_us_x10(stats.interval_max);
    uint32_t latency_avg_us = cycles_to_us_x10(latency_avg_cycles);
    uint32_t latency_max_us = cycles_to_us_x10(stats.latency_max);

    LOGI("stats", "%-14s E post=%lu.%lu/s done=%lu.%lu/s drop=%lu ovsz=%lu tmo=%lu qmax=%u exec(us) avg/min/max=%lu.%lu/%lu.%lu/%lu.%lu "
                  "int(us) min/max=%lu.%lu/%lu.%lu lat(us) avg/max=%lu.%lu/%lu.%lu stk=%luw",
         task->name, X10(post_rate), X10(done_rate), (unsigned long)drop_delta, (unsigned long)oversize_delta,
         (unsigned long)timeout_delta, stats.qmax,
         X10(exec_avg_us), X10(exec_min_us), X10(exec_max_us),
         X10(interval_min_us), X10(interval_max_us),
         X10(latency_avg_us), X10(latency_max_us), (unsigned long)stack_free_words);
}

void task_stats_dump(void)
{
    uint32_t now_ms = os_time_ms();

    cycles_per_us = task_stats_cycles_per_us();
    window_ms = now_ms - last_dump_ms;
    last_dump_ms = now_ms;
    LOGI("stats", "---- window %lu ms (cpu %lu MHz) ----", (unsigned long)window_ms, (unsigned long)cycles_per_us);
    period_foreach(dump_period, NULL);
    event_foreach(dump_event, NULL);
}
#endif

#ifdef CONFIG_SHELL
/* shell 命令 stats：打印所有任务（周期任务、事件任务）从上次触发到现在的运行统计，然后清零重新统计：
 * 周期任务的实测频率、执行时间、周期抖动、overrun、栈余量；事件任务的投递/处理/丢弃数、唤醒延迟等（第 8.6 节）。
 * CONFIG_TASK_STATS = n 时只打印一条提示。 */
static int shell_task_stats(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    task_stats_dump();
    return 0;
}
SHELL_CMD(stats, shell_task_stats, print_and_clear_all_task_stats)
#endif
