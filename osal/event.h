#ifndef OSAL_EVENT_H__
#define OSAL_EVENT_H__
#include "osal/osal.h"
#include "osal/freertos/osal_event_port.h"
#include "osal/task_stats.h"

/* 事件驱动任务：任务平时阻塞在自己的消息队列上，有消息才被唤醒。
 * 所有"数据来了要处理"的场合（串口、CAN、USB、中断引脚……）都用这一套：
 *   中断里  event_post_isr(&task, data, len)   把数据拷进队列元素，并唤醒任务
 *   任务里  fn(argument, data, len)                 拿到的是任务自己这一份拷贝，处理期间不会被改写
 * 超时：timeout_ms 非 0 时，等不到消息就以 data == NULL 调用一次 fn（用来做离线判断）。 */

enum
{
    EVENT_QUEUE  = 0,   /* 队列满时丢弃新消息，dropped 计数 +1 */
    EVENT_LATEST = 1,   /* 邮箱：深度必须为 1，新消息覆盖旧消息，只保留最新一条 */
};

#define EVENT_HDR_SIZE  6u  /* 每个队列元素前面是 uint16_t 长度 + uint32_t 投递时间戳，后面是 max_len 字节数据 */

struct event_task
{
    const char *name;
    void      (*fn)(void *argument, const uint8_t *data, uint16_t len);
    void       *argument;
    uint32_t    timeout_ms;                  /* 0 = 一直等，不产生超时调用 */
    uint16_t    max_len;                     /* 单条消息数据最大字节数 */
    uint16_t    depth;                       /* 队列深度 */
    os_priority_e priority;
    uint16_t    stack_words;                 /* 由 os_stack_e 换算出的字数，只有后端用。栈里要放 max_len 字节的消息缓冲，选档时要算上 */
    uint8_t     mode;                        /* EVENT_QUEUE / EVENT_LATEST */
    volatile uint32_t posted;
    volatile uint32_t handled;
    volatile uint32_t timeouts;
    volatile uint32_t dropped;
    volatile uint32_t oversize;
    void       *os_mem[4];                   /* 后端静态存储：tcb / stack / 队列控制块 / 队列存储区 */
    void       *os;                          /* 后端句柄（队列） */
    void       *os_task;                     /* 后端的任务句柄，查栈余量用 */
    struct task_stats stats;                 /* 统计（第 8.6 节） */
};

/* 定义一个事件任务：_priority 填 OS_PRIORITY_xxx，_stack 填 OS_STACK_xxx，
 * _depth 填 OS_QUEUE_LEN_xxx，_timeout_ms 填 OS_TIMEOUT_xxx。编译期检查参数；指针放进 event_tasks 段，不需要任何注册调用 */
#define EVENT_TASK_DEFINE(_name, _priority, _stack, _mode, _max_len, _depth, _timeout_ms, _fn, _argument)  \
    _Static_assert((_priority) >= OS_PRIORITY_LOWEST && (_priority) <= OS_PRIORITY_HIGHEST,                  \
                   #_name ": priority must be an os_priority_e value");                                \
    _Static_assert((_max_len) >= 1 && (_max_len) <= 1024, #_name ": bad size");                        \
    _Static_assert(OS_QUEUE_LEN_IS_VALID(_depth), #_name ": depth must be an os_queue_len_e value");           \
    _Static_assert(OS_TIMEOUT_IS_VALID(_timeout_ms), #_name ": timeout must be an os_timeout_e value"); \
    _Static_assert((_mode) != EVENT_LATEST || (_depth) == OS_QUEUE_LEN_1, #_name ": EVENT_LATEST needs OS_QUEUE_LEN_1"); \
    _Static_assert((_stack) >= OS_STACK_SMALL && (_stack) <= OS_STACK_LARGE, \
                   #_name ": stack must be an os_stack_e value");                                      \
    OS_EVENT_STORAGE(_name, OS_STACK_WORDS(_stack), _depth, EVENT_HDR_SIZE + (_max_len))                          \
    struct event_task _name = { #_name, (_fn), (_argument), (_timeout_ms), (_max_len), (_depth), (_priority),   \
                                OS_STACK_WORDS(_stack), (_mode), 0, 0, 0, 0, 0, OS_EVENT_MEM(_name), 0, 0, { 0 } }; \
    static struct event_task *const _name##_reg                                                         \
        __attribute__((section("event_tasks"), used)) = &_name

int32_t event_start_all(void);   /* framework_init 最先调用：创建全部队列和任务，之后中断里才能 post */
/* 只能在中断里调用。len > max_len 时丢弃并计 oversize；队列满计 dropped */
int32_t event_post_isr(struct event_task *task, const void *data, uint16_t len);
void    event_foreach(void (*callback)(struct event_task *task, void *context), void *context);
void    event_entry(void *task);    /* 后端创建任务时使用的入口 */

/* ===== 后端接口：换 OS 只需实现这几个（osal/freertos/osal_freertos.c）===== */
int32_t os_event_start(struct event_task *task);
int32_t os_event_wait(struct event_task *task, uint8_t *item, uint32_t timeout_ms);   /* ERR_OK / ERR_TIMEOUT */
int32_t os_event_post_isr(struct event_task *task, const uint8_t *item);              /* 内部负责唤醒后的切换 */
uint32_t os_event_pending_isr(struct event_task *task);                               /* 队列里现有的消息数（中断里用） */
#endif
