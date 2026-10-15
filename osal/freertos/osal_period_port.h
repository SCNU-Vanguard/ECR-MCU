#ifndef OSAL_PERIOD_PORT_H__
#define OSAL_PERIOD_PORT_H__
#include "osal/freertos/osal_port.h"
/* 每个周期任务的静态存储：TCB、栈、二值信号量。需要 configSUPPORT_STATIC_ALLOCATION=1 */
#define OS_PERIOD_STORAGE(_name, _stack_words)                                       \
    static StaticTask_t      _name##_tcb;                                            \
    static StackType_t       _name##_stack[_stack_words];                            \
    static StaticSemaphore_t _name##_sem;
#define OS_PERIOD_MEM(_name)                                                         \
    {                                                                                \
        &_name##_tcb,                                                                \
        _name##_stack,                                                               \
        &_name##_sem,                                                                \
    }
#endif
