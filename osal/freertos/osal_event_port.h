#ifndef OSAL_EVENT_PORT_H__
#define OSAL_EVENT_PORT_H__
#include "osal/freertos/osal_port.h"
/* 每个事件任务的静态存储：TCB、栈、队列控制块、队列存储区。需要 configSUPPORT_STATIC_ALLOCATION=1 */
#define OS_EVENT_STORAGE(_name, _stack_words, _depth, _item_size)                         \
    static StaticTask_t  _name##_tcb;                                                     \
    static StackType_t   _name##_stack[_stack_words];                                     \
    static StaticQueue_t _name##_queue_ctrl;                                              \
    static uint8_t       _name##_queue_buf[(_depth) * (_item_size)] __attribute__((aligned(4)));
#define OS_EVENT_MEM(_name)                                                               \
    {                                                                                     \
        &_name##_tcb,                                                                     \
        _name##_stack,                                                                    \
        &_name##_queue_ctrl,                                                              \
        _name##_queue_buf,                                                                \
    }
/* 框架优先级 0..EVENT_PRIORITY_MAX-1（0 最高），与周期任务共用同一套数值 */
#define EVENT_PRIORITY_MAX      24
#endif
