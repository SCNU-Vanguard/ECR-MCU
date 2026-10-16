#ifndef BSP_HAL_H__
#define BSP_HAL_H__
/* 按 MCU 家族选 HAL 总头文件：bsp 里其它文件统一 include 本文件，换 F407 时这里加一个分支 */
#if defined(STM32H723xx)
#include "stm32h7xx_hal.h"
#elif defined(STM32F407xx)
#include "stm32f4xx_hal.h"
#else
#error "unknown MCU: define STM32H723xx or STM32F407xx in build/mcu/<mcu>.mk"
#endif

/* DMA 能不能访问这个地址：DMA1/DMA2 访问不到 DTCM（默认 .data/.bss/栈所在）和 ITCM。
 * H723：AXI SRAM（0x24000000，512 KB）和 D2 SRAM（0x30000000，32 KB）可以；SRAM4（0x38000000）只有 BDMA 能访问，所以不算。
 * 缓冲区用 SRAM_NONCACHEABLE（component/common/types.h）声明，链接到 RAM_D1 */
#if defined(STM32H723xx)
#include <stdbool.h>
#include <stdint.h>
static inline bool bsp_dma_address_ok(const void *address)
{
    uint32_t value = (uint32_t)address;

    return (value >= 0x24000000u && value < 0x24080000u) || (value >= 0x30000000u && value < 0x30008000u);
}
#endif
#endif
