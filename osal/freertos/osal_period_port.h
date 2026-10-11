#ifndef OSAL_PERIOD_PORT_H__
#define OSAL_PERIOD_PORT_H__
#include "osal/freertos/osal_port.h"
/* 每个周期任务的静态存储：TCB、栈、二值信号量。需要 configSUPPORT_STATIC_ALLOCATION=1 */
#define OS_PERIOD_STORAGE(_n, _stk)                                                  \
    static StaticTask_t      _n##_tcb;                                               \
    static StackType_t       _n##_stack[_stk];                                       \
    static StaticSemaphore_t _n##_sem;
#define OS_PERIOD_MEM(_n)   { &_n##_tcb, _n##_stack, &_n##_sem }
/* 框架优先级 0..PERIOD_PRIO_MAX-1（0 最高）。上限 24：保证周期任务都高于 CubeIDE 的 defaultTask(24) */
#define PERIOD_PRIO_MAX     24
#endif
