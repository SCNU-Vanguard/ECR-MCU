#ifndef COMPONENT_PID_H__
#define COMPONENT_PID_H__
#include <stdint.h>
#include <stdbool.h>

/* PID 控制器：与硬件无关的纯算法，从旧仓库 algorithm/controller/PID 移植。
 * 三种计算：角度位置式（±pi 回绕）、增量式。
 * 单位由调用者决定（比如角度环 rad -> rad/s，速度环 rad/s -> 电流原始值）。 */

/* 参数：限幅为 0 表示该项输出恒为 0，所以必须显式给出 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float kf;                    /* 增量式的目标变化前馈系数，不用填 0 */
    float output_limit;          /* 输出限幅 ±output_limit */
    float integral_limit;        /* 积分限幅 ±integral_limit */
    float feedforward_limit;     /* kf 前馈项限幅（只有增量式用到） */
    float dead_band;             /* 死区：|误差| 不超过它时输出清零 */
} pid_config_t;

typedef struct
{
    pid_config_t config;
    float error;
    float last_error;
    float previous_error;
    float last_target;
    float integral;
    float output;
} pid_controller_t;

void  pid_init(pid_controller_t *pid, const pid_config_t *config);
void  pid_clear(pid_controller_t *pid);                              /* 清零内部状态，保留参数 */

/* 角度位置式：积分累加；目标和误差都回绕到 [-pi, pi]，所以只能用于单圈角度（rad），不能拿来算电流、速度这类线性量 */
float pid_angle_compute(pid_controller_t *pid, float measure, float target);
/* 线性位置式：P + 累计 I（限幅）+ 误差差分 D，输出限幅；用于温度这类线性量。ki、kd 已按固定调用周期整定，内部不乘除 dt。\r\n * 死区内输出清零，积分保留 */
float pid_position_compute(pid_controller_t *pid, float measure, float target);
/* 增量式：每次输出 = 上次输出 + 增量（速度环用） */
float pid_increment_compute(pid_controller_t *pid, float measure, float target);

/* 把角度回绕到 [-pi, pi) */
float pid_wrap_pi(float angle);
#endif
