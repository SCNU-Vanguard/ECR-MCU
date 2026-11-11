#ifndef COMMON_TYPES_H__
#define COMMON_TYPES_H__
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define ARRAY_SIZE(array)   (sizeof(array) / sizeof((array)[0]))

/* 协议载荷按 1 字节排布：成员之间和末尾都不插入填充，sizeof 就是线上的字节数。 */
#define BYTE_PACKED __attribute__((packed, aligned(1)))

/* 函数放进 .RamFunc。链接脚本把它载到 ITCM（0x00000000），memory_init() 从 Flash 拷过去。
 * noinline：内联之后段属性留在调用者里，函数体仍在 Flash。
 * AXI SRAM（0x24000000 / 0x24040000）在 MPU 里禁止取指，不能把函数放进去。 */
#define RAM_FUNC __attribute__((section(".RamFunc"), noinline))

/* 不可缓存数据：DMA 缓冲、软件队列、字节流重组、RTOS 堆。放在 RAM_D1（0x24000000，256 KB）。
 * DMA1/DMA2 访问不到 DTCM。memory_init() 在开 D-Cache 之前清零。 */
#define SRAM_NONCACHEABLE __attribute__((section(".sram_noncacheable"), aligned(32)))

/* 可缓存大表。放在 RAM（0x24040000，64 KB）。Flash 里留一份初值，memory_init() 在开 D-Cache 之前拷过去。 */
#define SRAM_CACHEABLE    __attribute__((section(".sram_cacheable"), aligned(8)))
#endif
