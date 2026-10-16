#include <string.h>
#include "stm32h7xx.h"

/* 进 main 之前做两件事（constructor 优先级 101，是用户构造函数里最先跑的）：
 *   1. 把 .RamFunc 从 Flash 拷到 ITCM。ITCM 在 0x00000000，按 64 位做 ECC，没写过就读会报错，
 *      所以先用 STRD 把 64 KB 整段写成 0，再拷镜像。VTOR 先指到 Flash 里的向量表，
 *      否则异常向量会落到刚清掉的 ITCM 上。
 *   2. 把 .sram_data（大表）从 Flash 拷到 AXI SRAM，再清 .sram_bss。
 *      启动文件只处理 DTCM 上的 .data / .bss。
 * 不改 Cube 生成的 startup_stm32h723xx.s。此时中断还没开，I-Cache 也还没开。 */

extern const uint32_t g_pfnVectors[];
extern uint8_t _itcm_start[];
extern uint8_t _itcm_end[];
extern uint8_t _sramfunc[];
extern uint8_t _eramfunc[];
extern uint8_t _siramfunc[];
extern uint8_t _ssram_data[];
extern uint8_t _esram_data[];
extern uint8_t _sisram_data[];
extern uint8_t _ssram_bss[];
extern uint8_t _esram_bss[];

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

    memcpy(_ssram_data, _sisram_data, (size_t)(_esram_data - _ssram_data));
    memset(_ssram_bss, 0, (size_t)(_esram_bss - _ssram_bss));
}
