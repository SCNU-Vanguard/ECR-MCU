#ifndef OSAL_TASK_STATS_H__
#define OSAL_TASK_STATS_H__
#include "component/common/types.h"

/* 任务统计：每个周期任务、事件任务的描述符里都带一份，随任务一起静态分配（注册任务时自动创建）。
 * 时间用 Cortex-M7 的 DWT 周期计数器测量（480 MHz 下分辨率约 2 ns），单位是"周期数"，
 * 打印时再换算成微秒。统计的是墙钟时间：任务执行期间被更高优先级任务或中断抢占的时间也算在内。
 *
 * 统计窗口 = 从上一次 task_stats_dump() 到这一次。dump 之后窗口内的统计量全部清零，重新开始。 */
struct task_stats
{
    /* ---- 窗口内的统计量（dump 后清零）---- */
    uint32_t run_count;                      /* 窗口内处理的次数 */
    uint32_t exec_last;                      /* 执行时间（周期数）：最近一次 */
    uint32_t exec_min;
    uint32_t exec_max;
    uint64_t exec_sum;
    uint32_t interval_min;                   /* 相邻两次"开始执行"的间隔（周期数）：周期抖动 / 事件到达节奏 */
    uint32_t interval_max;
    uint32_t latency_max;                    /* 唤醒延迟：放行/投递 -> 任务开始执行（周期数） */
    uint64_t latency_sum;
    uint16_t qmax;                           /* 事件队列里同时积压过的最大消息数（周期任务不用） */
    /* ---- 跨窗口保持 ---- */
    uint32_t release;                        /* 周期任务：tick 中断最近一次放行的时间戳 */
    uint32_t last_start;                     /* 上一次开始执行的时间戳 */
    bool     started;
    uint32_t base[5];                        /* 上次 dump 时的累计计数，用来求窗口内的增量 */
};

#ifdef CONFIG_TASK_STATS
void     task_stats_hw_init(void);                                       /* 打开 DWT 周期计数器，framework_init 调用 */
uint32_t task_stats_now(void);                                           /* 读周期计数器 */
uint32_t task_stats_cycles_per_us(void);                                 /* 每微秒多少个周期 */
uint32_t task_stats_begin(struct task_stats *stats, uint32_t stamp);     /* 任务开始执行时调用，返回开始时刻 */
void     task_stats_end(struct task_stats *stats, uint32_t start_cycles);/* 任务执行完调用 */
void     task_stats_reset(struct task_stats *stats);                     /* 清零窗口内的统计量（保留 base 和 last_start） */
#else
#define task_stats_hw_init()                      ((void)0)
#define task_stats_now()                          0u
#define task_stats_cycles_per_us()                1u
#define task_stats_begin(stats, stamp)            0u
#define task_stats_end(stats, start_cycles)       ((void)(start_cycles))
#define task_stats_reset(stats)                   ((void)0)
#endif
#endif
