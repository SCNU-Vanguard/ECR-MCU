#ifndef BSP_TICK_H__
#define BSP_TICK_H__
#include <stdint.h>
/* 自启动毫秒数。基于 HAL 时基，中断/任务/调度器启动前都能调用 */
uint32_t bsp_tick_ms(void);

/* 自启动微秒数（64 位，不回绕）。基于 DWT 周期计数器，中断/任务里都能调用，用来给传感器中断打时间戳。
 * 分辨率约 1 个 CPU 周期；每毫秒 HAL 时基中断里会同步一次，所以 32 位周期计数器回绕不会丢时间。
 * 在 INIT_BSP_STAGE1（bsp_tick_init）之后才可用 */
uint64_t bsp_tick_us(void);

/* 周期计数器（CPU 时钟，32 位，约 8.9 s 回绕一次，用无符号减法算差值不受影响） */
uint32_t bsp_cycle_count(void);
uint32_t bsp_cycle_reenable_count(void);           /* 周期计数器被外部（调试器）关掉后重新打开的次数，正常为 0 */
uint32_t bsp_cycle_freq_hz(void);                  /* CPU 时钟频率，即 bsp_cycle_count 每秒走多少 */
/* 距 *last_cycles 过了多少秒，同时把 *last_cycles 更新为当前值（周期任务里算 dt 用） */
float    bsp_cycle_delta_s(uint32_t *last_cycles);
#endif
