/* 覆盖 HAL 的弱符号 HAL_IncTick()：CubeIDE 把 HAL 时基配在 TIM7（1 kHz，优先级 15），
 * 每次 TIM7 更新中断都会调用 HAL_IncTick()。我们在这里顺带驱动周期任务，
 * 这样不用改任何 CubeIDE 生成的文件，也不用改 FreeRTOS 配置。 */
#include "bsp/include/bsp_hal.h"
#include "bsp/tick/bsp_tick.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "osal/period.h"

_Static_assert(HAL_TICK_FREQ_DEFAULT == HAL_TICK_FREQ_1KHZ, "OS_TICK_HZ assumes 1 kHz HAL tick");

/* 64 位周期时间线：每次同步把"自上次以来走过的周期数"累加进去。
 * 只要每 8 秒内同步一次就不会丢；HAL_IncTick 每毫秒同步一次。
 * 同步在 PRIMASK 临界区里做，因为 bsp_tick_us 可能在高优先级中断里打断 HAL_IncTick。 */
static uint32_t timeline_last_cycles;
static uint64_t timeline_total_cycles;

static void timeline_sync(void)
{
    uint32_t now_cycles = DWT->CYCCNT;

    timeline_total_cycles += (uint32_t)(now_cycles - timeline_last_cycles);
    timeline_last_cycles = now_cycles;
}

static volatile uint32_t cycle_counter_reenable_count;

/* 保证 DWT 周期计数器在跑。调试器连接/断开/复位时可能把 DEMCR.TRCENA 或 DWT_CTRL 清掉，计数器就会冻住，
 * 时间戳、任务统计、CPU 占用都会变成 0。这里每次发现被关就重新打开（不清零，计数值保持连续） */
static void cycle_counter_ensure(void)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0u || (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->LAR = 0xC5ACCE55u;                          /* Cortex-M7 的 DWT 默认锁着，要先解锁 */
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        cycle_counter_reenable_count++;
    }
}

uint32_t bsp_cycle_reenable_count(void)
{
    return cycle_counter_reenable_count;
}

static int32_t bsp_tick_init(void)
{
    cycle_counter_ensure();
    DWT->CYCCNT = 0u;
    timeline_last_cycles = 0u;
    timeline_total_cycles = 0u;
    cycle_counter_reenable_count = 0u;
    return ERR_OK;
}
INIT_BSP_STAGE1(bsp_tick_init);

void HAL_IncTick(void)
{
    uint32_t irq_state = __get_PRIMASK();

    uwTick += (uint32_t)uwTickFreq;      /* 保留 HAL 原有行为（HAL_Delay / HAL_GetTick 依赖它） */
    __disable_irq();
    cycle_counter_ensure();
    timeline_sync();
    __set_PRIMASK(irq_state);
    period_tick_isr();
}

uint32_t bsp_tick_ms(void)
{
    return HAL_GetTick();
}

uint64_t bsp_tick_us(void)
{
    uint32_t irq_state = __get_PRIMASK();
    uint64_t total_cycles;

    __disable_irq();
    timeline_sync();
    total_cycles = timeline_total_cycles;
    __set_PRIMASK(irq_state);
    return total_cycles / (SystemCoreClock / 1000000u);
}

uint32_t bsp_cycle_count(void)
{
    return DWT->CYCCNT;
}

uint32_t bsp_cycle_freq_hz(void)
{
    return SystemCoreClock;
}

float bsp_cycle_delta_s(uint32_t *last_cycles)
{
    uint32_t now_cycles = DWT->CYCCNT;
    uint32_t delta_cycles = now_cycles - *last_cycles;

    *last_cycles = now_cycles;
    return (float)delta_cycles / (float)SystemCoreClock;
}
