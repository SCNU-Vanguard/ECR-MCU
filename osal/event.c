#include <string.h>
#include "osal/event.h"

extern struct event_task *const __start_event_tasks[];
extern struct event_task *const __stop_event_tasks[];

void event_entry(void *arg)
{
    struct event_task *t = arg;
    uint8_t item[EVENT_HDR_SIZE + t->max_len];                 /* 任务自己的接收缓冲，fn 拿到的指针指向这里 */
    for (;;) {
        uint32_t wait = (t->timeout_ms == 0u) ? OS_WAIT_FOREVER : t->timeout_ms;
        if (os_event_wait(t, item, wait) == ERR_OK) {
            uint16_t len;
            uint32_t stamp;
            memcpy(&len, item, sizeof len);
            memcpy(&stamp, item + sizeof len, sizeof stamp);          /* 中断里投递时的时间戳 */
            if (len > t->max_len) { len = t->max_len; }
            t->handled++;
            uint32_t t0 = task_stats_begin(&t->st, stamp);            /* 记唤醒延迟、处理间隔 */
            t->fn(t->arg, item + EVENT_HDR_SIZE, len);
            task_stats_end(&t->st, t0);                               /* 记执行时间 */
        } else {
            t->timeouts++;
            t->fn(t->arg, NULL, 0);                            /* 超时：data == NULL */
        }
    }
}

int32_t event_start_all(void)
{
    int32_t ret = ERR_OK;
    for (struct event_task *const *p = __start_event_tasks; p < __stop_event_tasks; p++) {
        if (os_event_start(*p) != ERR_OK) { ret = ERR_NOMEM; }
    }
    return ret;
}

int32_t event_post_isr(struct event_task *t, const void *data, uint16_t len)
{
    if (t->os == NULL) { t->dropped++; return ERR_NODEV; }     /* 队列还没创建 */
    if (len > t->max_len) { t->oversize++; return ERR_INVAL; }
    uint8_t item[EVENT_HDR_SIZE + t->max_len];                 /* 中断栈上的临时元素（队列按固定大小拷贝） */
    uint32_t stamp = task_stats_now();
    memcpy(item, &len, sizeof len);
    memcpy(item + sizeof len, &stamp, sizeof stamp);
    if (len != 0u) { memcpy(item + EVENT_HDR_SIZE, data, len); }
    if (os_event_post_isr(t, item) != ERR_OK) { t->dropped++; return ERR_BUSY; }
    t->posted++;
#ifdef CONFIG_TASK_STATS
    uint32_t pend = os_event_pending_isr(t);                   /* 队列水位 */
    if (pend > t->st.qmax) { t->st.qmax = (uint16_t)pend; }
#endif
    return ERR_OK;
}

void event_foreach(void (*cb)(struct event_task *t, void *ctx), void *ctx)
{
    for (struct event_task *const *p = __start_event_tasks; p < __stop_event_tasks; p++) { cb(*p, ctx); }
}
