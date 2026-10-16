#include <string.h>
#include "stm32h7xx.h"

/* 进 main 之前做两件事（constructor 优先级 101，是用户构造函数里最先跑的）：
 *   1. 把 .RamFunc 从 Flash 拷到 ITCM。ITCM 在 0x00000000，按 64 位做 ECC，没写过就读会报错，
 *      所以先用 STRD 把 64 KB 整段写成 0，再拷镜像。VTOR 先指到 Flash 里的向量表，
 *      否则异常向量会落到刚清掉的 ITCM 上。
 *   2. 把 .sram_cacheable（大表）从 Flash 拷到 RAM，清 RAM_D1 上的 .sram_noncacheable 和 RAM_D2 上的 USB 缓冲。
 *      启动文件只处理 DTCM 上的 .data / .bss。MPU 只在 Cube 的 MPU_Config() 里配。
 * 不改 Cube 生成的 startup_stm32h723xx.s。此时中断还没开，I-Cache 也还没开。 */

extern const uint32_t g_pfnVectors[];
extern uint8_t _itcm_start[];
extern uint8_t _itcm_end[];
extern uint8_t _sramfunc[];
extern uint8_t _eramfunc[];
extern uint8_t _siramfunc[];
extern uint8_t _ssram_cacheable[];
extern uint8_t _esram_cacheable[];
extern uint8_t _sisram_cacheable[];
extern uint8_t _ssram_noncacheable[];
extern uint8_t _esram_noncacheable[];
extern uint8_t _susb_dma[];
extern uint8_t _eusb_dma[];

/* 64 位写，一次写满一个 ECC 字。用固定寄存器，避免编译器把指针分到 r2/r3 上和清零寄存器撞车 */
static void itcm_clear(uint32_t start, uint32_t end)
{
    __asm volatile(
        "mov r0, %[start]\n"
        "mov r1, %[end]\n"
        "movs r2, #0\n"
        "movs r3, #0\n"
        "1:\n"
        "strd r2, r3, [r0], #8\n"
        "cmp r0, r1\n"
        "bcc 1b\n"
        :
        : [start] "r"(start), [end] "r"(end)
        : "r0", "r1", "r2", "r3", "cc", "memory");
}

static void memory_init(void) __attribute__((constructor(101)));
static void memory_init(void)
{
    SCB->VTOR = (uint32_t)g_pfnVectors;
    __DSB();
    __ISB();

    itcm_clear((uint32_t)_itcm_start, (uint32_t)_itcm_end);
    memcpy(_sramfunc, _siramfunc, (size_t)(_eramfunc - _sramfunc));
    __DSB();
    __ISB();

    memcpy(_ssram_cacheable, _sisram_cacheable, (size_t)(_esram_cacheable - _ssram_cacheable));
    memset(_ssram_noncacheable, 0, (size_t)(_esram_noncacheable - _ssram_noncacheable));

    /* RAM_D2 时钟复位后是关的，DATA_IN_D2_SRAM 又没开。时钟关着写等于没写，
     * hpcd.State 会是上电随机值。HAL_PCD_Init 看到不是 RESET 就跳过 MspInit，
     * USB 时钟不开，CoreReset 一直等不到 AHBIDL，最后进 Error_Handler。 */
    RCC->AHB2ENR |= RCC_AHB2ENR_D2SRAM1EN | RCC_AHB2ENR_D2SRAM2EN;
    (void)RCC->AHB2ENR;
    memset(_susb_dma, 0, (size_t)(_eusb_dma - _susb_dma));
}
