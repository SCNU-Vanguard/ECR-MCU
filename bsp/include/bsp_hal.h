#ifndef BSP_HAL_H__
#define BSP_HAL_H__
/* 按 MCU 家族选 HAL 总头文件：bsp 里其它文件统一 include 本文件，换 F407 时这里加一个分支 */
#if defined(STM32H723xx)
#include "stm32h7xx_hal.h"
#elif defined(STM32F407xx)
#include "stm32f4xx_hal.h"
#else
#error "unknown MCU: define STM32H723xx or STM32F407xx in build/<mcu>/<mcu>.mk"
#endif

/* DMA 能不能访问 [address, address + length) 这整块缓冲区：DMA1/DMA2 访问不到 DTCM（默认 .data/.bss/栈所在）和 ITCM。
 * H723：只认 MPU 里配成"不可缓存"、且 DMA 能到的区域（见 platform main.c 的 MPU 配置）：
 *   AXI SRAM 前 256 KB（0x24000000，不可缓存；之后 64 KB 是 .sram_cacheable，可缓存，DMA 不做 Cache 维护所以不算）
 *   D2 SRAM（0x30000000，32 KB，不可缓存）
 * SRAM4（0x38000000）只有 BDMA 能访问，所以不算。
 * 缓冲区用 SRAM_NONCACHEABLE（component/common/types.h）声明，链接到 RAM_D1 */
#if defined(STM32H723xx)
#include <stdbool.h>
#include <stdint.h>
static inline bool bsp_dma_address_ok(const void *address, uint32_t length)
{
    uint32_t start = (uint32_t)address;

    if (length == 0u)
    {
        return false;
    }
    /* 用 "长度 <= 区域末尾 - 起点" 的写法，避免 start + length 回绕 */
    if (start >= 0x24000000u && start < 0x24040000u)
    {
        return length <= 0x24040000u - start;
    }
    if (start >= 0x30000000u && start < 0x30008000u)
    {
        return length <= 0x30008000u - start;
    }
    return false;
}
#endif
#endif
