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

uint32_t task_stats_now(void)           { return DWT->CYCCNT; }
uint32_t task_stats_cycles_per_us(void) { return SystemCoreClock / 1000000u; }

void task_stats_reset(struct task_stats *s)
{
    s->n = 0u;
    s->exec_last = 0u; s->exec_min = 0xFFFFFFFFu; s->exec_max = 0u; s->exec_sum = 0u;
    s->int_min = 0xFFFFFFFFu; s->int_max = 0u;
    s->lat_max = 0u; s->lat_sum = 0u;
    s->qmax = 0u;
}

uint32_t task_stats_begin(struct task_stats *s, uint32_t stamp)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t lat = now - stamp;                       /* 无符号减法：计数器回绕也正确 */
    s->lat_sum += lat;
    if (lat > s->lat_max) { s->lat_max = lat; }
    if (s->started) {
        uint32_t itv = now - s->last_start;
        if (itv < s->int_min) { s->int_min = itv; }
        if (itv > s->int_max) { s->int_max = itv; }
    }
    s->started = true;
    s->last_start = now;
    return now;
}

void task_stats_end(struct task_stats *s, uint32_t t0)
{
    uint32_t e = DWT->CYCCNT - t0;
    s->exec_last = e;
    if (e < s->exec_min) { s->exec_min = e; }
    if (e > s->exec_max) { s->exec_max = e; }
    s->exec_sum += e;
    s->n++;
}
#endif
