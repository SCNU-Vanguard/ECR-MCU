#ifndef BSP_CPU_H__
#define BSP_CPU_H__
#include <stdbool.h>

/* 关掉全部中断（PRIMASK），不恢复。致命路径用；普通临界区用 os_irq_save()。 */
void bsp_irq_disable(void);
/* 调试器是否接在这颗核上 */
bool bsp_debugger_attached(void);
void bsp_breakpoint(void);
/* 芯片复位，不返回 */
void bsp_system_reset(void) __attribute__((noreturn));
#endif
