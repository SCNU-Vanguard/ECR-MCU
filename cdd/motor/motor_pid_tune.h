#ifndef CDD_MOTOR_PID_TUNE_H__
#define CDD_MOTOR_PID_TUNE_H__

#include "bsp/include/bsp_ids.h"

/* 电机 PID 自动整定钩子（复用 UART_LOG 串口：Shell 收命令 + PID: 文本行回传数据）。
 * 未开启 CONFIG_COMPONENT_SHELL 或未进入整定模式时零开销、不影响正常任务控制。 */
#ifdef CONFIG_COMPONENT_SHELL
void motor_pid_tune_pre_compute(can_id_e bus);
void motor_pid_tune_post_compute(can_id_e bus);
#else
static inline void motor_pid_tune_pre_compute(can_id_e bus)  { (void)bus; }
static inline void motor_pid_tune_post_compute(can_id_e bus) { (void)bus; }
#endif

#endif
