#include "component/common/taskstat.h"
#include "component/log/log.h"
#include "osal/period.h"
#include "osal/event.h"
#include <stdio.h>
#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"
#endif

#ifndef CONFIG_OSAL_TASK_STATS
void task_stats_dump(void)
{
    LOGW("CONFIG_OSAL_TASK_STATS is off");
}
#else

static uint32_t cycles_per_us;      /* 每微秒的周期数 */
static uint32_t window_ms;          /* 本次统计窗口长度（毫秒） */
static uint32_t last_dump_ms;       /* 上次触发的时刻 */

/* 周期数换成微秒，四舍五入。 */
static uint32_t cycles_to_us(uint32_t cycles)
{
    return (uint32_t)(((uint64_t)cycles + cycles_per_us / 2u) / cycles_per_us);
}

static uint32_t rate_per_second(uint32_t count)
{
    if (window_ms == 0u)
    {
        return 0u;
    }
    return (uint32_t)(((uint64_t)count * 1000u + window_ms / 2u) / window_ms);
}

/* 各列宽度固定，周期任务和事件任务的执行时间、延迟、栈才能上下对齐。 */
static void dump_line(const char *name, char kind, const char *freq, const char *rate,
                      const char *flag, const char *exec, int have_latency, uint32_t latency_us,
                      uint32_t stack_free, const char *extra)
{
    char latency[8];

    if (have_latency)
    {
        (void)snprintf(latency, sizeof latency, "%4lu", (unsigned long)latency_us);
    }
    else
    {
        latency[0] = '\0';
    }
    LOGI("%-20s %c %6s %6s %7s %13s lat=%4s stk=%4lu%s",
         name, kind, freq, rate, flag, exec, latency, (unsigned long)stack_free, extra);
}

/* 计数为 0 时不追加，避免把后面的列顶歪。 */
static void append_count(char *buf, size_t size, const char *label, uint32_t value)
{
    size_t used = 0;

    if (value == 0u)
    {
        return;
    }
    while (buf[used] != '\0')
    {
        used++;
    }
    if (used < size)
    {
        (void)snprintf(buf + used, size - used, " %s=%lu", label, (unsigned long)value);
    }
}

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

    char freq[8];
    char rate[8];
    char flag[12];
    char exec[16];
    uint32_t stack_free_words = os_task_stack_free(task->os_task);

    (void)snprintf(freq, sizeof freq, "%4luHz", (unsigned long)(OS_TICK_HZ / task->div));
    (void)snprintf(flag, sizeof flag, "ov=%4lu", (unsigned long)overrun_delta);
    if (stats.run_count == 0u)
    {
        dump_line(task->name, 'P', freq, "  idle", flag, "", 0, 0u, stack_free_words, "");
        return;
    }
    (void)snprintf(rate, sizeof rate, "%4lu/s", (unsigned long)rate_per_second(run_delta));
    (void)snprintf(exec, sizeof exec, "e=%4lu/%4luus",
                   (unsigned long)cycles_to_us((uint32_t)(stats.exec_sum / stats.run_count)),
                   (unsigned long)cycles_to_us(stats.exec_max));
    dump_line(task->name, 'P', freq, rate, flag, exec, 1, cycles_to_us(stats.latency_max),
              stack_free_words, "");
}

static void dump_event(struct event_task *task, void *context)
{
    (void)context;
    struct task_stats stats;
    uint32_t irq_key = os_irq_save();

    stats = task->stats;
    uint32_t post_delta = task->posted - task->stats.base[0];
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

    char rate[12];
    char flag[16];
    char exec[16];
    char extra[48];
    uint32_t stack_free_words = os_task_stack_free(task->os_task);

    extra[0] = '\0';
    append_count(extra, sizeof extra, "drop", drop_delta);
    append_count(extra, sizeof extra, "ovsz", oversize_delta);
    append_count(extra, sizeof extra, "tmo", timeout_delta);
    if (stats.run_count == 0u)
    {
        dump_line(task->name, 'E', "", "  idle", "", "", 0, 0u, stack_free_words, extra);
        return;
    }
    (void)snprintf(rate, sizeof rate, "%4lu/s", (unsigned long)rate_per_second(post_delta));
    (void)snprintf(flag, sizeof flag, " q=%4u", (unsigned)stats.qmax);
    (void)snprintf(exec, sizeof exec, "e=%4lu/%4luus",
                   (unsigned long)cycles_to_us((uint32_t)(stats.exec_sum / stats.run_count)),
                   (unsigned long)cycles_to_us(stats.exec_max));
    dump_line(task->name, 'E', "", rate, flag, exec, 1, cycles_to_us(stats.latency_max),
              stack_free_words, extra);
}

void task_stats_dump(void)
{
    uint32_t now_ms = os_time_ms();

    cycles_per_us = task_stats_cycles_per_us();
    window_ms = now_ms - last_dump_ms;
    last_dump_ms = now_ms;
    LOGI("---- %lu ms ----", (unsigned long)window_ms);
    period_foreach(dump_period, NULL);
    event_foreach(dump_event, NULL);
}
#endif

#ifdef CONFIG_COMPONENT_SHELL
/* shell 命令 stats：打印各任务从上一次到现在的运行情况，然后清零重计。
 * 周期任务：设定频率、实际频率、overrun、执行平均/最大、唤醒延迟最大、栈剩余。
 * 事件任务：投递频率、队列峰值、执行平均/最大、唤醒延迟最大、栈剩余。丢包、超长、超时为 0 时不打印。
 * CONFIG_OSAL_TASK_STATS = n 时只打印一条提示。 */
static int shell_task_stats(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    task_stats_dump();
    return 0;
}
SHELL_CMD(stats, shell_task_stats, print_and_clear_all_task_stats)
#endif
