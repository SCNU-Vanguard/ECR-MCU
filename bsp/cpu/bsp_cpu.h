#ifndef BSP_CPU_H__
#define BSP_CPU_H__
#include <stdbool.h>

/* GCC only. 0xF = SY，全系统屏障。"memory" 挡住编译器把两侧的访问互相穿过。
 * DMB 只保证顺序；DSB 等前面的显式访存完成；ISB 让后面的指令重新取指。
 * COMPILER_BARRIER 只挡编译器，不发硬件指令。 */
#define COMPILER_BARRIER()  __asm volatile ("" ::: "memory")
#define DMB()               __asm volatile ("dmb 0xF" ::: "memory")
#define DSB()               __asm volatile ("dsb 0xF" ::: "memory")
#define ISB()               __asm volatile ("isb 0xF" ::: "memory")

/* 关掉全部中断（PRIMASK），不恢复。致命路径用；普通临界区用 os_irq_save()。 */
void bsp_irq_disable(void);
/* 调试器是否接在这颗核上 */
bool bsp_debugger_attached(void);
void bsp_breakpoint(void);
/* 芯片复位，不返回 */
void bsp_system_reset(void) __attribute__((noreturn));
#endif
