#ifndef OSAL_PORT_H__
#define OSAL_PORT_H__
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* os_stack_e -> 栈字数（StackType_t，Cortex-M 上 4 字节一个字）。编译期常量，可以直接当数组长度 */
#define OS_STACK_WORDS(_stack)                                    \
    ((_stack) == OS_STACK_SMALL  ? 256u :                         \
     (_stack) == OS_STACK_MEDIUM ? 512u : 1024u)
#endif
