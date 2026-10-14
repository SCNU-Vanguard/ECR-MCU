#ifndef COMMON_TYPES_H__
#define COMMON_TYPES_H__
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define ARRAY_SIZE(array)   (sizeof(array) / sizeof((array)[0]))

/* DMA 缓冲区：放进 .dma_nc 段（链接到 AXI SRAM，MPU 已配成不可缓存）。
 * 不能放 DTCM（默认 .bss/.data 所在）：DMA1/DMA2 访问不到 DTCM。 */
#define DMA_NC          __attribute__((section(".dma_nc"), aligned(32)))
#endif
