#ifndef OSAL_PERIOD_H__
#define OSAL_PERIOD_H__
#include "osal/osal.h"
#include "osal/freertos/osal_period_port.h"
#include "osal/task_stats.h"

/* 周期任务描述符：全部静态分配。计时全部在 tick 中断里完成（period_tick_isr），
 * 任务体只做 "take 信号量 -> 执行 -> 再 take"，不用 vTaskDelay。 */
struct period_task
{
    const char *name;
    void      (*fn)(void *argument);              /* 每个周期调用一次，必须返回 */
    void       *argument;
    uint32_t    div;                         /* OS_TICK_HZ / freq */
    uint32_t    count;                       /* 分频计数：只在 tick 中断里读写 */
    volatile uint32_t overrun;               /* 上个周期还没被取走，这次释放失败 = 丢了一个周期 */
    volatile uint32_t runs;
    os_priority_e priority;
    uint16_t    stack_words;                 /* 由 os_stack_e 换算出的字数，只有后端用 */
    void       *os_mem[3];                   /* 后端静态存储：tcb / stack / sem */
    void       *os;                          /* 后端句柄（信号量） */
    void       *os_task;                     /* 后端的任务句柄，查栈余量用 */
    struct task_stats stats;                 /* 统计（第 8.6 节） */
};

/* 定义一个周期任务：_freq_hz 填 OS_FREQ_xxHZ，_priority 填 OS_PRIORITY_xxx，_stack 填 OS_STACK_xxx，编译期检查；指针放进 period_tasks 段，无需任何注册调用 */
#define PERIOD_TASK_DEFINE(_name, _freq_hz, _priority, _stack, _fn, _argument)                       \
    _Static_assert(OS_FREQ_IS_VALID(_freq_hz), #_name ": freq must be an os_freq_e value");                    \
    _Static_assert((_priority) >= OS_PRIORITY_LOWEST && (_priority) <= OS_PRIORITY_HIGHEST,                 \
                   #_name ": priority must be an os_priority_e value");                                \
    _Static_assert((_stack) >= OS_STACK_SMALL && (_stack) <= OS_STACK_LARGE, \
                   #_name ": stack must be an os_stack_e value");                                      \
    OS_PERIOD_STORAGE(_name, OS_STACK_WORDS(_stack))                                                        \
    struct period_task _name = { #_name, (_fn), (_argument), OS_TICK_HZ / (_freq_hz), 0, 0, 0,         \
                                 (_priority), OS_STACK_WORDS(_stack), OS_PERIOD_MEM(_name), 0, 0, { 0 } };    \
    static struct period_task *const _name##_reg                                                  \
        __attribute__((section("period_tasks"), used)) = &_name

int32_t period_start_all(void);   /* 初始化完成后调一次：创建全部任务，之后 tick 才开始放行 */
void    period_tick_isr(void);    /* 在 OS_TICK_HZ 的 tick 中断里调用（见 bsp/tick）           */
void    period_entry(void *task); /* 后端创建任务时使用的入口 */
/* 遍历全部周期任务（调试/统计用） */
void    period_foreach(void (*callback)(struct period_task *task, void *context), void *context);

/* ===== 后端接口：换 OS 只需实现这四个（osal/freertos/osal_freertos.c）===== */
int32_t os_period_start(struct period_task *task);
void    os_period_wait(struct period_task *task);
int32_t os_period_release_isr(struct period_task *task, int32_t *woken);
void    os_period_yield_isr(int32_t woken);
#endif
