#ifndef OSAL_EVENT_H__
#define OSAL_EVENT_H__
#include "osal/osal.h"
#include "osal/freertos/osal_event_port.h"
#include "osal/task_stats.h"

/* 事件驱动任务：任务平时阻塞在自己的消息队列上，有消息才被唤醒。
 * 所有"数据来了要处理"的场合（串口、CAN、USB、中断引脚……）都用这一套：
 *   中断里  event_post_isr(&task, data, len)   把数据拷进队列元素，并唤醒任务
 *   任务里  fn(arg, data, len)                 拿到的是任务自己这一份拷贝，处理期间不会被改写
 * 超时：timeout_ms 非 0 时，等不到消息就以 data == NULL 调用一次 fn（用来做离线判断）。 */

enum { EVENT_QUEUE = 0,     /* 队列满时丢弃新消息，dropped 计数 +1 */
       EVENT_LATEST = 1 };  /* 邮箱：深度必须为 1，新消息覆盖旧消息，只保留最新一条 */

#define EVENT_HDR_SIZE  6u  /* 每个队列元素前面是 uint16_t 长度 + uint32_t 投递时间戳，后面是 max_len 字节数据 */

struct event_task {
    const char *name;
    void      (*fn)(void *arg, const uint8_t *data, uint16_t len);
    void       *arg;
    uint32_t    timeout_ms;                  /* 0 = 一直等，不产生超时调用 */
    uint16_t    max_len;                     /* 单条消息数据最大字节数 */
    uint16_t    depth;                       /* 队列深度 */
    uint16_t    prio;                        /* 0..EVENT_PRIO_MAX-1，数值小优先级高 */
    uint16_t    stack_words;                 /* 栈（字）：要包含 max_len 字节的消息缓冲，再加 fn 自己的需要 */
    uint8_t     mode;                        /* EVENT_QUEUE / EVENT_LATEST */
    volatile uint32_t posted, handled, timeouts, dropped, oversize;
    void       *os_mem[4];                   /* 后端静态存储：tcb / stack / 队列控制块 / 队列存储区 */
    void       *os;                          /* 后端句柄（队列） */
    void       *os_task;                     /* 后端的任务句柄，查栈余量用 */
    struct task_stats st;                    /* 统计（第 8.6 节） */
};

/* 定义一个事件任务：编译期检查参数；指针放进 event_tasks 段，不需要任何注册调用 */
#define EVENT_TASK_DEFINE(_n, _prio, _stk, _mode, _maxlen, _depth, _timeout_ms, _fn, _arg)    \
    _Static_assert((_prio) >= 0 && (_prio) < EVENT_PRIO_MAX, #_n ": prio out of range");      \
    _Static_assert((_depth) >= 1 && (_maxlen) >= 1 && (_maxlen) <= 1024, #_n ": bad size");   \
    _Static_assert((_mode) != EVENT_LATEST || (_depth) == 1, #_n ": EVENT_LATEST needs depth 1"); \
    OS_EVENT_STORAGE(_n, _stk, _depth, EVENT_HDR_SIZE + (_maxlen))                            \
    struct event_task _n = { #_n, (_fn), (_arg), (_timeout_ms), (_maxlen), (_depth), (_prio), \
                             (_stk), (_mode), 0, 0, 0, 0, 0, OS_EVENT_MEM(_n), 0, 0, { 0 } };          \
    static struct event_task *const _n##_reg                                                  \
        __attribute__((section("event_tasks"), used)) = &_n

int32_t event_start_all(void);   /* framework_init 最先调用：创建全部队列和任务，之后中断里才能 post */
/* 只能在中断里调用。len > max_len 时丢弃并计 oversize；队列满计 dropped */
int32_t event_post_isr(struct event_task *t, const void *data, uint16_t len);
void    event_foreach(void (*cb)(struct event_task *t, void *ctx), void *ctx);
void    event_entry(void *t);    /* 后端创建任务时使用的入口 */

/* ===== 后端接口：换 OS 只需实现这三个（osal/freertos/osal_freertos.c）===== */
int32_t os_event_start(struct event_task *t);
int32_t os_event_wait(struct event_task *t, uint8_t *item, uint32_t timeout_ms);   /* ERR_OK / ERR_TIMEOUT */
int32_t os_event_post_isr(struct event_task *t, const uint8_t *item);              /* 内部负责唤醒后的切换 */
uint32_t os_event_pending_isr(struct event_task *t);                               /* 队列里现有的消息数（中断里用） */
#endif
