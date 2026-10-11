#ifndef COMMON_TASKSTAT_H__
#define COMMON_TASKSTAT_H__
/* 手动触发的任务统计：把所有周期任务、事件任务从上一次触发到现在的统计打印到日志，然后清零。
 * 框架不会自动调用它；想什么时候看，就在什么地方调（命令、按键、调试器 call、测试任务……）。
 * 必须在任务里调用（不能在中断里）：它要读 FreeRTOS 时间、打日志。
 * 需要 CONFIG_TASK_STATS = y；为 n 时打印一行提示。 */
void task_stats_dump(void);
#endif
