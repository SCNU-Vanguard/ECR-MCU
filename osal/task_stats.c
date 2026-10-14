#include "osal/task_stats.h"

#ifdef CONFIG_TASK_STATS
#include "osal/osal.h"
#include CMSIS_device_header            /* DWT、CoreDebug、SystemCoreClock */

void task_stats_hw_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 打开 DWT/ITM 的时钟 */
    DWT->LAR = 0xC5ACCE55u;                           /* Cortex-M7：先解锁才能写 DWT 寄存器 */
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t task_stats_now(void)
{
    return DWT->CYCCNT;
}

uint32_t task_stats_cycles_per_us(void)
{
    return SystemCoreClock / 1000000u;
}

void task_stats_reset(struct task_stats *stats)
{
    stats->run_count = 0u;
    stats->exec_last = 0u;
    stats->exec_min = 0xFFFFFFFFu;
    stats->exec_max = 0u;
    stats->exec_sum = 0u;
    stats->interval_min = 0xFFFFFFFFu;
    stats->interval_max = 0u;
    stats->latency_max = 0u;
    stats->latency_sum = 0u;
    stats->qmax = 0u;
}

uint32_t task_stats_begin(struct task_stats *stats, uint32_t stamp)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t latency = now - stamp;                   /* 无符号减法：计数器回绕也正确 */

    stats->latency_sum += latency;
    if (latency > stats->latency_max)
    {
        stats->latency_max = latency;
    }
    if (stats->started)
    {
        uint32_t interval = now - stats->last_start;

        if (interval < stats->interval_min)
        {
            stats->interval_min = interval;
        }
        if (interval > stats->interval_max)
        {
            stats->interval_max = interval;
        }
    }
    stats->started = true;
    stats->last_start = now;
    return now;
}

void task_stats_end(struct task_stats *stats, uint32_t start_cycles)
{
    uint32_t exec_cycles = DWT->CYCCNT - start_cycles;

    stats->exec_last = exec_cycles;
    if (exec_cycles < stats->exec_min)
    {
        stats->exec_min = exec_cycles;
    }
    if (exec_cycles > stats->exec_max)
    {
        stats->exec_max = exec_cycles;
    }
    stats->exec_sum += exec_cycles;
    stats->run_count++;
}
#endif
