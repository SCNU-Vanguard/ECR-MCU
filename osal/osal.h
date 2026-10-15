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

/* 周期任务允许的频率：只有这 10 档，且都能整除 OS_TICK_HZ。PERIOD_TASK_DEFINE 在编译期检查，别的值编译不过 */
typedef enum
{
    OS_FREQ_1HZ    = 1,
    OS_FREQ_2HZ    = 2,
    OS_FREQ_5HZ    = 5,
    OS_FREQ_10HZ   = 10,
    OS_FREQ_20HZ   = 20,
    OS_FREQ_50HZ   = 50,
    OS_FREQ_100HZ  = 100,
    OS_FREQ_200HZ  = 200,
    OS_FREQ_500HZ  = 500,
    OS_FREQ_1000HZ = 1000,
} os_freq_e;

#define OS_FREQ_IS_VALID(_freq)                                                          \
    ((_freq) == OS_FREQ_1HZ   || (_freq) == OS_FREQ_2HZ   || (_freq) == OS_FREQ_5HZ   ||  \
     (_freq) == OS_FREQ_10HZ  || (_freq) == OS_FREQ_20HZ  || (_freq) == OS_FREQ_50HZ  ||  \
     (_freq) == OS_FREQ_100HZ || (_freq) == OS_FREQ_200HZ || (_freq) == OS_FREQ_500HZ ||  \
     (_freq) == OS_FREQ_1000HZ)

/* 任务优先级：周期任务和事件任务共用这 5 档，具体数值由各 OS 后端映射（osal/freertos/osal_freertos.c），上层不接触 */
typedef enum
{
    OS_PRIORITY_LOWEST,
    OS_PRIORITY_LOW,
    OS_PRIORITY_MEDIUM,
    OS_PRIORITY_HIGH,
    OS_PRIORITY_HIGHEST,
} os_priority_e;

/* 任务栈大小：3 档，具体字数由各 OS 后端决定（osal/freertos/osal_port.h 里的 OS_STACK_WORDS），上层不接触。
 * 选档依据：SMALL 只做简单运算和外设操作；MEDIUM 有解码、队列消息缓冲；LARGE 有命令行、格式化输出等较深的调用 */
typedef enum
{
    OS_STACK_SMALL,
    OS_STACK_MEDIUM,
    OS_STACK_LARGE,
} os_stack_e;

/* 二值信号量 */
typedef struct
{
    SemaphoreHandle_t handle;
    StaticSemaphore_t storage;
} os_sem_t;

typedef struct
{
    SemaphoreHandle_t handle;
    StaticSemaphore_t storage;
} os_mutex_t;

bool    os_in_isr(void);
/* CPU 占用统计：在空闲钩子里调用。统计原理见 osal/freertos/cpu_usage.c；CONFIG_OSAL_CPU_USAGE 没开时是空操作 */
#ifdef CONFIG_OSAL_CPU_USAGE
void    os_idle_hook(void);
#else
#define os_idle_hook()  ((void)0)
#endif
/* OS 检测到任务栈溢出时由后端的钩子调用，不返回：记下任务名，有调试器就停在断点，否则复位。
 * 不能打日志（此时 OS 数据结构已不可信），任务名留在 os_overflow_task_name 里供调试器查看 */
extern volatile char os_overflow_task_name[16];
void    os_stack_overflow_handler(const char *task_name) __attribute__((noreturn));
/* OS 动态堆分配失败时由后端的钩子调用，处理方式同上，任务名位置记为 "heap" */
void    os_malloc_failed_handler(void) __attribute__((noreturn));
bool    os_running(void);
/* 任务/中断都能用的短临界区：屏蔽优先级数值 >= configMAX_SYSCALL_INTERRUPT_PRIORITY 的中断 */
uint32_t os_irq_save(void);
void     os_irq_restore(uint32_t key);

int32_t os_sem_init(os_sem_t *sem);
int32_t os_sem_take(os_sem_t *sem, uint32_t timeout_ms);
void    os_sem_give(os_sem_t *sem);
int32_t os_sem_give_isr(os_sem_t *sem);       /* 中断里用；返回 ERR_BUSY 表示上一次还没被取走 */

int32_t os_mutex_init(os_mutex_t *mutex);
int32_t os_mutex_lock(os_mutex_t *mutex, uint32_t timeout_ms);
void    os_mutex_unlock(os_mutex_t *mutex);

void     os_delay_ms(uint32_t delay_ms);
uint32_t os_time_ms(void);                  /* 自启动毫秒数（FreeRTOS tick） */
uint32_t os_task_stack_free(void *task);    /* 任务栈历史最小剩余（字）；task 是后端的任务句柄 */
#endif
