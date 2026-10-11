#ifndef OSAL_EVENT_PORT_H__
#define OSAL_EVENT_PORT_H__
#include "osal/freertos/osal_port.h"
/* 每个事件任务的静态存储：TCB、栈、队列控制块、队列存储区。需要 configSUPPORT_STATIC_ALLOCATION=1 */
#define OS_EVENT_STORAGE(_n, _stk, _depth, _item)                                         \
    static StaticTask_t  _n##_tcb;                                                        \
    static StackType_t   _n##_stack[_stk];                                                \
    static StaticQueue_t _n##_q;                                                          \
    static uint8_t       _n##_qbuf[(_depth) * (_item)] __attribute__((aligned(4)));
#define OS_EVENT_MEM(_n)    { &_n##_tcb, _n##_stack, &_n##_q, _n##_qbuf }
/* 框架优先级 0..EVENT_PRIO_MAX-1（0 最高），与周期任务共用同一套数值 */
#define EVENT_PRIO_MAX      24
#endif
