/* 覆盖 HAL 的弱符号 HAL_IncTick()：CubeIDE 把 HAL 时基配在 TIM7（1 kHz，优先级 15），
 * 每次 TIM7 更新中断都会调用 HAL_IncTick()。我们在这里顺带驱动周期任务，
 * 这样不用改任何 CubeIDE 生成的文件，也不用改 FreeRTOS 配置。 */
#include "bsp/include/bsp_hal.h"
#include "bsp/tick/bsp_tick.h"
#include "osal/period.h"

_Static_assert(HAL_TICK_FREQ_DEFAULT == HAL_TICK_FREQ_1KHZ, "OS_TICK_HZ assumes 1 kHz HAL tick");

void HAL_IncTick(void)
{
    uwTick += (uint32_t)uwTickFreq;      /* 保留 HAL 原有行为（HAL_Delay / HAL_GetTick 依赖它） */
    period_tick_isr();
}

uint32_t bsp_tick_ms(void) { return HAL_GetTick(); }
