#include "osal/osal.h"
#include "osal/period.h"
#include "osal/event.h"

#include CMSIS_device_header            /* __get_IPSR；FreeRTOSConfig.h 里定义为 "stm32h7xx.h" */

static inline TickType_t ms_to_ticks(uint32_t time_ms)
{
    return (time_ms == OS_WAIT_FOREVER) ? portMAX_DELAY : (TickType_t)pdMS_TO_TICKS(time_ms);
}

bool os_in_isr(void)
{
    return __get_IPSR() != 0u;
}

bool os_running(void)
{
    return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
}

uint32_t os_irq_save(void)
{
    return portSET_INTERRUPT_MASK_FROM_ISR();
}

void os_irq_restore(uint32_t key)
{
    portCLEAR_INTERRUPT_MASK_FROM_ISR(key);
}

int32_t os_sem_init(os_sem_t *sem)
{
    sem->handle = xSemaphoreCreateBinaryStatic(&sem->storage);
    return (sem->handle != NULL) ? ERR_OK : ERR_NOMEM;
}

int32_t os_sem_take(os_sem_t *sem, uint32_t timeout_ms)
{
    return (xSemaphoreTake(sem->handle, ms_to_ticks(timeout_ms)) == pdTRUE) ? ERR_OK : ERR_TIMEOUT;
}

void os_sem_give(os_sem_t *sem)
{
    (void)xSemaphoreGive(sem->handle);
}

int32_t os_sem_give_isr(os_sem_t *sem)
{
    BaseType_t woken = pdFALSE;
    BaseType_t give_result = xSemaphoreGiveFromISR(sem->handle, &woken);

    portYIELD_FROM_ISR(woken);
    return (give_result == pdTRUE) ? ERR_OK : ERR_BUSY;
}

int32_t os_mutex_init(os_mutex_t *mutex)
{
    mutex->handle = xSemaphoreCreateMutexStatic(&mutex->storage);
    return (mutex->handle != NULL) ? ERR_OK : ERR_NOMEM;
}

int32_t os_mutex_lock(os_mutex_t *mutex, uint32_t timeout_ms)
{
    return (xSemaphoreTake(mutex->handle, ms_to_ticks(timeout_ms)) == pdTRUE) ? ERR_OK : ERR_TIMEOUT;
}

void os_mutex_unlock(os_mutex_t *mutex)
{
    (void)xSemaphoreGive(mutex->handle);
}

void os_delay_ms(uint32_t delay_ms)
{
    vTaskDelay((delay_ms == 0u) ? 1u : (TickType_t)pdMS_TO_TICKS(delay_ms));
}

uint32_t os_time_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

uint32_t os_task_stack_free(void *task)
{
    return (task != NULL) ? (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)task) : 0u;
}

/* ---------- 周期任务后端（period.c 要求的四个函数）---------- */
/* 框架优先级（0 最高）-> FreeRTOS 优先级（数值大最高） */
#define PERIOD_PRIORITY_MAP(priority)   ((UBaseType_t)(configMAX_PRIORITIES - 1 - (priority)))

int32_t os_period_start(struct period_task *task)
{
    SemaphoreHandle_t semaphore = xSemaphoreCreateBinaryStatic((StaticSemaphore_t *)task->os_mem[2]);

    if (semaphore == NULL)
    {
        return ERR_NOMEM;
    }
    task->os = semaphore;
    TaskHandle_t handle = xTaskCreateStatic(period_entry, task->name, task->stack_words, task,
                                            PERIOD_PRIORITY_MAP(task->priority),
                                            (StackType_t *)task->os_mem[1], (StaticTask_t *)task->os_mem[0]);

    task->os_task = handle;
    return (handle != NULL) ? ERR_OK : ERR_NOMEM;
}

void os_period_wait(struct period_task *task)
{
    (void)xSemaphoreTake((SemaphoreHandle_t)task->os, portMAX_DELAY);
}

int32_t os_period_release_isr(struct period_task *task, int32_t *woken)
{
    BaseType_t higher_priority_woken = pdFALSE;
    BaseType_t give_result = xSemaphoreGiveFromISR((SemaphoreHandle_t)task->os, &higher_priority_woken);  /* 二值信号量已满返回 pdFAIL */

    if (higher_priority_woken != pdFALSE)
    {
        *woken = 1;
    }
    return (give_result == pdTRUE) ? ERR_OK : ERR_BUSY;
}

void os_period_yield_isr(int32_t woken)
{
    portYIELD_FROM_ISR(woken != 0 ? pdTRUE : pdFALSE);                   /* 一次 tick 只请求一次切换 */
}

/* ---------- 事件任务后端（event.c 要求的几个函数）---------- */
int32_t os_event_start(struct event_task *task)
{
    QueueHandle_t queue = xQueueCreateStatic(task->depth, EVENT_HDR_SIZE + task->max_len,
                                             (uint8_t *)task->os_mem[3], (StaticQueue_t *)task->os_mem[2]);

    if (queue == NULL)
    {
        return ERR_NOMEM;
    }
    task->os = queue;
    TaskHandle_t handle = xTaskCreateStatic(event_entry, task->name, task->stack_words, task,
                                            PERIOD_PRIORITY_MAP(task->priority),
                                            (StackType_t *)task->os_mem[1], (StaticTask_t *)task->os_mem[0]);

    task->os_task = handle;
    return (handle != NULL) ? ERR_OK : ERR_NOMEM;
}

int32_t os_event_wait(struct event_task *task, uint8_t *item, uint32_t timeout_ms)
{
    return (xQueueReceive((QueueHandle_t)task->os, item, ms_to_ticks(timeout_ms)) == pdTRUE) ? ERR_OK : ERR_TIMEOUT;
}

int32_t os_event_post_isr(struct event_task *task, const uint8_t *item)
{
    BaseType_t woken = pdFALSE;
    BaseType_t send_result = (task->mode == EVENT_LATEST)
        ? xQueueOverwriteFromISR((QueueHandle_t)task->os, item, &woken)      /* 深度 1：覆盖旧消息 */
        : xQueueSendFromISR((QueueHandle_t)task->os, item, &woken);          /* 满了返回 errQUEUE_FULL */

    portYIELD_FROM_ISR(woken);
    return (send_result == pdTRUE) ? ERR_OK : ERR_BUSY;
}

uint32_t os_event_pending_isr(struct event_task *task)
{
    return (uint32_t)uxQueueMessagesWaitingFromISR((QueueHandle_t)task->os);
}
