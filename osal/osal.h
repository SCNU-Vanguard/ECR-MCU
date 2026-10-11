#ifndef OSAL_H__
#define OSAL_H__
/* 对 FreeRTOS 原生 API 的极薄封装。上层代码只 include 本文件，不直接 include FreeRTOS.h。
 * 全部静态分配（对象存储在调用者手里），不使用 FreeRTOS 堆。 */
#include "component/common/types.h"
#include "component/common/err.h"
#include "osal/freertos/osal_port.h"

#define OS_WAIT_FOREVER  0xFFFFFFFFu
#define OS_NO_WAIT       0u

/* 周期任务的时间基准（Hz）：必须等于 period_tick_isr() 的调用频率。
 * 本工程由 HAL 时基 TIM7 的 1 kHz 中断驱动（见 bsp/tick）。 */
#define OS_TICK_HZ       1000u

typedef struct { SemaphoreHandle_t h; StaticSemaphore_t buf; } os_sem_t;     /* 二值信号量 */
typedef struct { SemaphoreHandle_t h; StaticSemaphore_t buf; } os_mutex_t;

bool    os_in_isr(void);
bool    os_running(void);
/* 任务/中断都能用的短临界区：屏蔽优先级数值 >= configMAX_SYSCALL_INTERRUPT_PRIORITY 的中断 */
uint32_t os_irq_save(void);
void     os_irq_restore(uint32_t key);

int32_t os_sem_init(os_sem_t *s);
int32_t os_sem_take(os_sem_t *s, uint32_t timeout_ms);
void    os_sem_give(os_sem_t *s);
int32_t os_sem_give_isr(os_sem_t *s);       /* 中断里用；返回 ERR_BUSY 表示上一次还没被取走 */

int32_t os_mutex_init(os_mutex_t *m);
int32_t os_mutex_lock(os_mutex_t *m, uint32_t timeout_ms);
void    os_mutex_unlock(os_mutex_t *m);

void     os_delay_ms(uint32_t ms);
uint32_t os_time_ms(void);                  /* 自启动毫秒数（FreeRTOS tick） */
uint32_t os_task_stack_free(void *task);    /* 任务栈历史最小剩余（字）；task 是后端的任务句柄 */
#endif
