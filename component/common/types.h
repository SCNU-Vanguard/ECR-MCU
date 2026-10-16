#ifndef COMMON_TYPES_H__
#define COMMON_TYPES_H__
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define ARRAY_SIZE(array)   (sizeof(array) / sizeof((array)[0]))

/* DMA 缓冲区：放进 .dma_nc 段（链接到 AXI SRAM，MPU 已配成不可缓存）。
 * 不能放 DTCM（默认 .bss/.data 所在）：DMA1/DMA2 访问不到 DTCM。
 * 启动不清这片，使用前自己初始化。 */
#define DMA_NC          __attribute__((section(".dma_nc"), aligned(32)))

/* 大块 CPU 缓冲（软件队列、字节流重组、协议栈收发缓冲、RTOS 堆）：放进 .sram_bss，同一片 AXI SRAM。
 * 这片不可缓存，CPU 和 DMA 都能直接访问。任务栈和热路径上的小状态留在 DTCM。
 * 启动时由 memory_init() 整段清零。 */
#define SRAM_BSS        __attribute__((section(".sram_bss"), aligned(8)))

/* 大表：放进 .sram_data。Flash 里留一份初值，memory_init() 拷到 AXI SRAM。 */
#define SRAM_DATA       __attribute__((section(".sram_data"), aligned(8)))
#endif
