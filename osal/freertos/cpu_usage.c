/* CPU 占用统计：靠 FreeRTOS 空闲钩子测"空闲任务实际跑了多久"，占用 = 1 - 空闲时间 / 总时间。
 *
 * 空闲钩子每循环一次调用一次，两次调用之间只隔几十个周期（循环本身的开销）。如果两次调用的间隔超过 IDLE_GAP_MAX，
 * 说明空闲任务在这期间被别的任务或较长的中断抢占了，这段时间不算空闲；间隔很短就算空闲。
 * 因此：
 *   - 比 2 us 短的中断（DRDY、DMA 完成等）算在空闲里，使占用率略偏低；比 2 us 长的中断算占用。
 *   - 和空闲任务同优先级的任务会被当成空闲（本框架最低优先级任务高于空闲，不受影响）。
 *   - 总时间用 FreeRTOS tick（毫秒），空闲时间用 DWT 周期计数器累计成 64 位，所以统计窗口多长都不会回绕。
 * 依赖 DWT 周期计数器已打开（bsp_tick_init 里打开）。
 *
 * shell：cpu 打印"自上次 cpu 命令以来"和"自启动以来"的占用率；第一次执行时两者相同。 */
#include "osal/osal.h"
#include CMSIS_device_header

#define IDLE_GAP_MAX_US     2u          /* 空闲钩子两次调用间隔超过这个值，就认为空闲任务被抢占过 */

static uint32_t last_hook_cycles;
static uint32_t idle_gap_max_cycles;
static volatile uint64_t idle_cycles;                 /* 累计空闲周期数，只有空闲任务写 */

/* 窗口起点：上一次 cpu 命令时的累计空闲周期数和 tick */
static uint64_t window_start_idle_cycles;
static uint32_t window_start_tick_ms;

void os_idle_hook(void)
{
    uint32_t now_cycles = DWT->CYCCNT;
    uint32_t gap_cycles = now_cycles - last_hook_cycles;

    if (idle_gap_max_cycles == 0u)
    {
        idle_gap_max_cycles = (SystemCoreClock / 1000000u) * IDLE_GAP_MAX_US;
    }
    last_hook_cycles = now_cycles;
    if (gap_cycles <= idle_gap_max_cycles)
    {
        uint32_t irq_key = os_irq_save();             /* 64 位加法不是原子的：读侧在别的任务里，写到一半被切走会读到撕裂的值，所以两侧都关中断 */

        idle_cycles += gap_cycles;
        os_irq_restore(irq_key);
    }
}

static uint64_t read_idle_cycles(void)
{
    uint32_t irq_key = os_irq_save();
    uint64_t value = idle_cycles;

    os_irq_restore(irq_key);
    return value;
}

/* 空闲时间占比换算成 0.1% 单位的占用率（0..1000） */
static uint32_t usage_permille(uint64_t idle_delta_cycles, uint32_t elapsed_ms)
{
    uint64_t total_cycles = (uint64_t)elapsed_ms * (SystemCoreClock / 1000u);

    if (total_cycles == 0u)
    {
        return 0u;
    }
    if (idle_delta_cycles >= total_cycles)
    {
        return 0u;
    }
    return (uint32_t)(1000u - (uint32_t)((idle_delta_cycles * 1000u) / total_cycles));
}

#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"
#include "FreeRTOS.h"
#include "task.h"

static int shell_cpu(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    uint64_t idle_now = read_idle_cycles();
    uint32_t tick_now = (uint32_t)xTaskGetTickCount();
    uint32_t window_ms = tick_now - window_start_tick_ms;
    uint32_t window_permille = usage_permille(idle_now - window_start_idle_cycles, window_ms);
    uint32_t boot_permille = usage_permille(idle_now, tick_now);

    shell_out("cpu: window %lu ms usage %lu.%lu%%, since boot %lu.%lu%%\r\n", (unsigned long)window_ms,
              (unsigned long)(window_permille / 10u), (unsigned long)(window_permille % 10u),
              (unsigned long)(boot_permille / 10u), (unsigned long)(boot_permille % 10u));
    window_start_idle_cycles = idle_now;
    window_start_tick_ms = tick_now;
    return 0;
}
SHELL_CMD(cpu, shell_cpu, print_cpu_usage_since_last_call)
#endif
