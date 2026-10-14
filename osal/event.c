#include <string.h>
#include "osal/event.h"

extern struct event_task *const __start_event_tasks[];
extern struct event_task *const __stop_event_tasks[];

void event_entry(void *argument)
{
    struct event_task *task = argument;
    uint8_t item[EVENT_HDR_SIZE + task->max_len];                 /* 任务自己的接收缓冲，fn 拿到的指针指向这里 */

    for (;;)
    {
        uint32_t wait_ms = (task->timeout_ms == 0u) ? OS_WAIT_FOREVER : task->timeout_ms;

        if (os_event_wait(task, item, wait_ms) == ERR_OK)
        {
            uint16_t length;
            uint32_t stamp;

            memcpy(&length, item, sizeof length);
            memcpy(&stamp, item + sizeof length, sizeof stamp);          /* 中断里投递时的时间戳 */
            if (length > task->max_len)
            {
                length = task->max_len;
            }
            task->handled++;
            uint32_t start_cycles = task_stats_begin(&task->stats, stamp);   /* 记唤醒延迟、处理间隔 */
            task->fn(task->argument, item + EVENT_HDR_SIZE, length);
            task_stats_end(&task->stats, start_cycles);                      /* 记执行时间 */
        }
        else
        {
            task->timeouts++;
            task->fn(task->argument, NULL, 0);                            /* 超时：data == NULL */
        }
    }
}

int32_t event_start_all(void)
{
    int32_t ret = ERR_OK;

    for (struct event_task *const *entry = __start_event_tasks; entry < __stop_event_tasks; entry++)
    {
        if (os_event_start(*entry) != ERR_OK)
        {
            ret = ERR_NOMEM;
        }
    }
    return ret;
}

int32_t event_post_isr(struct event_task *task, const void *data, uint16_t len)
{
    if (task->os == NULL)                                          /* 队列还没创建 */
    {
        task->dropped++;
        return ERR_NODEV;
    }
    if (len > task->max_len)
    {
        task->oversize++;
        return ERR_INVAL;
    }
    uint8_t item[EVENT_HDR_SIZE + task->max_len];                  /* 中断栈上的临时元素（队列按固定大小拷贝） */
    uint32_t stamp = task_stats_now();

    memcpy(item, &len, sizeof len);
    memcpy(item + sizeof len, &stamp, sizeof stamp);
    if (len != 0u)
    {
        memcpy(item + EVENT_HDR_SIZE, data, len);
    }
    if (os_event_post_isr(task, item) != ERR_OK)
    {
        task->dropped++;
        return ERR_BUSY;
    }
    task->posted++;
#ifdef CONFIG_OSAL_TASK_STATS
    uint32_t pending_count = os_event_pending_isr(task);           /* 队列水位 */

    if (pending_count > task->stats.qmax)
    {
        task->stats.qmax = (uint16_t)pending_count;
    }
#endif
    return ERR_OK;
}

void event_foreach(void (*callback)(struct event_task *task, void *context), void *context)
{
    for (struct event_task *const *entry = __start_event_tasks; entry < __stop_event_tasks; entry++)
    {
        callback(*entry, context);
    }
}
