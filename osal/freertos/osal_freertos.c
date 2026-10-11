#include "osal/osal.h"
#include "osal/period.h"
#include "osal/event.h"

#include CMSIS_device_header            /* __get_IPSR；FreeRTOSConfig.h 里定义为 "stm32h7xx.h" */

static inline TickType_t ms2tick(uint32_t ms)
{
    return (ms == OS_WAIT_FOREVER) ? portMAX_DELAY : (TickType_t)pdMS_TO_TICKS(ms);
}

bool os_in_isr(void)  { return __get_IPSR() != 0u; }
bool os_running(void) { return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED; }

uint32_t os_irq_save(void)               { return portSET_INTERRUPT_MASK_FROM_ISR(); }
void     os_irq_restore(uint32_t key)    { portCLEAR_INTERRUPT_MASK_FROM_ISR(key); }

int32_t os_sem_init(os_sem_t *s)
{
    s->h = xSemaphoreCreateBinaryStatic(&s->buf);
    return (s->h != NULL) ? ERR_OK : ERR_NOMEM;
}
int32_t os_sem_take(os_sem_t *s, uint32_t ms)
{
    return (xSemaphoreTake(s->h, ms2tick(ms)) == pdTRUE) ? ERR_OK : ERR_TIMEOUT;
}
void os_sem_give(os_sem_t *s) { (void)xSemaphoreGive(s->h); }
int32_t os_sem_give_isr(os_sem_t *s)
{
    BaseType_t woken = pdFALSE;
    BaseType_t ok = xSemaphoreGiveFromISR(s->h, &woken);
    portYIELD_FROM_ISR(woken);
    return (ok == pdTRUE) ? ERR_OK : ERR_BUSY;
}

int32_t os_mutex_init(os_mutex_t *m)
{
    m->h = xSemaphoreCreateMutexStatic(&m->buf);
    return (m->h != NULL) ? ERR_OK : ERR_NOMEM;
}
int32_t os_mutex_lock(os_mutex_t *m, uint32_t ms)
{
    return (xSemaphoreTake(m->h, ms2tick(ms)) == pdTRUE) ? ERR_OK : ERR_TIMEOUT;
}
void os_mutex_unlock(os_mutex_t *m) { (void)xSemaphoreGive(m->h); }

void     os_delay_ms(uint32_t ms) { vTaskDelay((ms == 0u) ? 1u : (TickType_t)pdMS_TO_TICKS(ms)); }
uint32_t os_time_ms(void)         { return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS); }
uint32_t os_task_stack_free(void *task) { return (task != NULL) ? (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)task) : 0u; }

/* ---------- 周期任务后端（period.c 要求的四个函数）---------- */
/* 框架优先级（0 最高）-> FreeRTOS 优先级（数值大最高） */
#define PERIOD_PRIO_MAP(n)   ((UBaseType_t)(configMAX_PRIORITIES - 1 - (n)))

int32_t os_period_start(struct period_task *t)
{
    SemaphoreHandle_t s = xSemaphoreCreateBinaryStatic((StaticSemaphore_t *)t->os_mem[2]);
    if (s == NULL) { return ERR_NOMEM; }
    t->os = s;
    TaskHandle_t h = xTaskCreateStatic(period_entry, t->name, t->stack_words, t,
                                       PERIOD_PRIO_MAP(t->prio),
                                       (StackType_t *)t->os_mem[1], (StaticTask_t *)t->os_mem[0]);
    t->os_task = h;
    return (h != NULL) ? ERR_OK : ERR_NOMEM;
}

void os_period_wait(struct period_task *t)
{
    (void)xSemaphoreTake((SemaphoreHandle_t)t->os, portMAX_DELAY);
}

int32_t os_period_release_isr(struct period_task *t, int32_t *woken)
{
    BaseType_t w = pdFALSE;
    BaseType_t r = xSemaphoreGiveFromISR((SemaphoreHandle_t)t->os, &w);  /* 二值信号量已满返回 pdFAIL */
    if (w != pdFALSE) { *woken = 1; }
    return (r == pdTRUE) ? ERR_OK : ERR_BUSY;
}

void os_period_yield_isr(int32_t woken)
{
    portYIELD_FROM_ISR(woken != 0 ? pdTRUE : pdFALSE);                   /* 一次 tick 只请求一次切换 */
}

/* ---------- 事件任务后端（event.c 要求的三个函数）---------- */
int32_t os_event_start(struct event_task *t)
{
    QueueHandle_t q = xQueueCreateStatic(t->depth, EVENT_HDR_SIZE + t->max_len,
                                         (uint8_t *)t->os_mem[3], (StaticQueue_t *)t->os_mem[2]);
    if (q == NULL) { return ERR_NOMEM; }
    t->os = q;
    TaskHandle_t h = xTaskCreateStatic(event_entry, t->name, t->stack_words, t,
                                       PERIOD_PRIO_MAP(t->prio),
                                       (StackType_t *)t->os_mem[1], (StaticTask_t *)t->os_mem[0]);
    t->os_task = h;
    return (h != NULL) ? ERR_OK : ERR_NOMEM;
}

int32_t os_event_wait(struct event_task *t, uint8_t *item, uint32_t timeout_ms)
{
    return (xQueueReceive((QueueHandle_t)t->os, item, ms2tick(timeout_ms)) == pdTRUE) ? ERR_OK : ERR_TIMEOUT;
}

int32_t os_event_post_isr(struct event_task *t, const uint8_t *item)
{
    BaseType_t woken = pdFALSE;
    BaseType_t ok = (t->mode == EVENT_LATEST)
        ? xQueueOverwriteFromISR((QueueHandle_t)t->os, item, &woken)      /* 深度 1：覆盖旧消息 */
        : xQueueSendFromISR((QueueHandle_t)t->os, item, &woken);          /* 满了返回 errQUEUE_FULL */
    portYIELD_FROM_ISR(woken);
    return (ok == pdTRUE) ? ERR_OK : ERR_BUSY;
}

uint32_t os_event_pending_isr(struct event_task *t)
{
    return (uint32_t)uxQueueMessagesWaitingFromISR((QueueHandle_t)t->os);
}
