#ifndef OSAL_PORT_H__
#define OSAL_PORT_H__
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Cortex-M / GCC：包含编译器屏障，不依赖 CMSIS，也无需启动调度器。
 * portMEMORY_BARRIER 只有编译器屏障，不能替代硬件 DMB。 */
static inline void os_memory_barrier(void)
{
    __asm volatile ("dmb sy" ::: "memory");
}

static inline void os_data_sync_barrier(void)
{
    __asm volatile ("dsb sy" ::: "memory");
}

static inline void os_instruction_sync_barrier(void)
{
    __asm volatile ("isb sy" ::: "memory");
}

/* os_stack_e -> 栈字数（StackType_t，Cortex-M 上 4 字节一个字）。编译期常量，可以直接当数组长度 */
#define OS_STACK_WORDS(_stack)                                    \
    ((_stack) == OS_STACK_SMALL  ? 256u :                         \
     (_stack) == OS_STACK_MEDIUM ? 512u : 1024u)
#endif
