#ifndef ROBOT_CFG_H__
#define ROBOT_CFG_H__
/* demo 的构型：这台机器人有哪些电机、各挂在哪条 CAN 总线上、ID 是多少、软件控制的 PID 是多少。
 * 这个文件只放数据，不放逻辑；定义变量和绑定的代码都在 robot_init.c，改机器人配置只需要改这里。
 * 框架不关心电机属于底盘还是云台，这里只列"电机 + 总线 + ID"，名字随便取（应用代码里直接用这个名字访问电机）。
 * 模块要不要编译、功能开不开，不在这里，在 robot/demo/demo.config 的 CONFIG_XXX = y/n。
 *
 * 每种电机有两张表，一个电机在两张表里各占一行，行首用同一个名字：
 *   1. 电机表（ROBOT_xxx_MOTOR_TABLE）：硬件信息，决定"电机挂在哪、怎么通信"。
 *   2. 控制表（ROBOT_xxx_CONTROL_TABLE）：软件控制的 PID 参数，决定"电机怎么控"。
 *      表里每行会生成一个 const motor_control_config_t，名字是 <电机名>_control，电机表最后一列用 &<电机名>_control 引用它；
 *      不用软件控制的电机（只想直接发原始值）不用写控制表，电机表最后一列填 NULL。
 * 表里没有电机 = 不注册任何电机。每个电机会生成一个全局变量（名字就是表里的名字），robot_init.c 在初始化时按表绑定到 CAN 总线。
 * 打开前确认总线上真有电机：经典 CAN 开了自动重传，发给无应答总线的帧会被不断重发，最终总线错误。 */
#include "bsp/include/bsp_ids.h"
#include "cdd/motor/motor.h"

/* ================= DJI 电机（M3508 / M2006 / GM6020） ================= */

/* 电机表，列含义：
 *   名字 | 总线 | 型号 | ID(拨码) | 反向 | 编码器零点偏移 0..8191 | 离线超时 ms | GM6020 控制量(电压/电流) | 控制配置指针(NULL=不用软件控制)
 * 例（M3508 的 ID 1、2 挂在 CAN_BUS0；GM6020 的 ID 5 挂在 CAN_BUS1，电压模式）：
 *   X(motor_a, CAN_BUS0, MOTOR_TYPE_M3508,  1, false, 0, 50, DJI_GM6020_VOLTAGE, &motor_a_control)
 *   X(motor_b, CAN_BUS0, MOTOR_TYPE_M3508,  2, true,  0, 50, DJI_GM6020_VOLTAGE, &motor_b_control)
 *   X(motor_c, CAN_BUS1, MOTOR_TYPE_GM6020, 5, false, 0, 50, DJI_GM6020_VOLTAGE, &motor_c_control) */
#define ROBOT_DJI_MOTOR_TABLE(X)                                                                                  \
    X(shoot_motor,   CAN_BUS0, MOTOR_TYPE_M3508,  1, false, 0, 50, DJI_GM6020_VOLTAGE, &shoot_motor_control)     \
    X(chassis_motor, CAN_BUS1, MOTOR_TYPE_GM6020, 1, false, 0, 50, DJI_GM6020_VOLTAGE, &chassis_motor_control)

/* 控制表，列含义（角度环、速度环各 8 个，顺序一致）：
 *   名字 | 角度环 kp | ki | kd | kf | 输出限幅 | 积分限幅 | 前馈限幅 | 死区 | 速度环（同上 8 项）| 角度反馈 | 速度反馈 | 速度前馈 | 转矩前馈
 * 单位：角度环 rad -> rad/s；速度环 rad/s -> 电流/电压原始值（DJI）。
 * 角度环是位置式（ki 累加进积分，积分限幅限的是累加值）；速度环是增量式（积分限幅限的是 ki*误差 这一项）。
 * 只用速度环的电机，角度环整行填 0。
 *
 * 最后 4 列是控制信号来源，每一项都是 float 指针，控制器每个周期读它指向的值：
 *   角度反馈 rad | 速度反馈 rad/s | 速度前馈 rad/s | 转矩前馈
 * 默认填 MOTOR_FEEDBACK_OWN（用电机自己的反馈）和 MOTOR_FEEDFORWARD_ZERO（前馈为 0）；
 * 要用外部信号时把它换成实际变量的地址（例如 &imu_yaw_rad），或者运行时用 motor_bind_xxx 改绑。
 * 这些是没有实测过的保守初值：电机会动得很弱甚至不动，现场用 shell 的 xxx_pid 命令调。 */
#define ROBOT_DJI_CONTROL_TABLE(X)                                                                                                                    \
    X(shoot_motor,    0.0f,  0.0f, 0.0f, 0.0f,    0.0f,    0.0f, 0.0f, 0.0f,    10.0f, 1.0f, 0.0f, 0.0f, 2000.0f,  500.0f, 0.0f, 0.0f,    MOTOR_FEEDBACK_OWN, MOTOR_FEEDBACK_OWN, MOTOR_FEEDFORWARD_ZERO, MOTOR_FEEDFORWARD_ZERO)   \
    X(chassis_motor,  5.0f,  0.0f, 0.0f, 0.0f,   10.0f,    0.0f, 0.0f, 0.0f,    30.0f, 3.0f, 0.0f, 0.0f, 4000.0f, 1000.0f, 0.0f, 0.0f,    MOTOR_FEEDBACK_OWN, MOTOR_FEEDBACK_OWN, MOTOR_FEEDFORWARD_ZERO, MOTOR_FEEDFORWARD_ZERO)

/* ================= DM 电机（DM4310 / DM8009P） ================= */

/* 电机表，列含义：
 *   名字 | 总线 | 型号 | tx_id(电机 CAN ID) | master_id(反馈帧 ID) | 硬件模式 | 反向 | p_max rad | v_max rad/s | t_max N·m | 离线超时 ms
 *        | MIT 用法(原生/软件转矩) | Kp 0..500 | Kd 0..5 | 速度上限 rad/s | 电流上限 0..1 | 控制配置指针(NULL=不用软件控制)
 * 例（MIT 软件转矩：Kp=Kd=0，转矩由软件级联 PID 算，和 2026 步兵/标准一致）：
 *   X(motor_d, CAN_BUS1, MOTOR_TYPE_DM4310, 1, 0x11, DM_MODE_MIT, false, 12.5f, 30.0f, 10.0f, 100, DM_MIT_SOFTWARE_TORQUE, 0.0f, 0.0f, 0.0f, 0.0f, &motor_d_control) */
#define ROBOT_DM_MOTOR_TABLE(X)                                                                                                                                       \
    X(gimbal_motor, CAN_BUS2, MOTOR_TYPE_DM4310, 1, 0x11, DM_MODE_MIT, false, 3.1415927f, 10.0f, 10.0f, 100, DM_MIT_SOFTWARE_TORQUE, 0.0f, 0.0f, 0.0f, 0.0f, &gimbal_motor_control)

/* 控制表，列含义同 DJI 的控制表；单位：角度环 rad -> rad/s；速度环 rad/s -> 转矩 N·m。
 * 没有实测过：速度环输出限幅 1 N·m（DM4310 的 t_max 是 10 N·m）。 */
#define ROBOT_DM_CONTROL_TABLE(X)                                                                                                                     \
    X(gimbal_motor,   5.0f,  0.0f, 0.0f, 0.0f,    5.0f,    0.0f, 0.0f, 0.0f,     0.2f, 0.02f, 0.0f, 0.0f,    1.0f,    0.2f, 0.0f, 0.0f,    MOTOR_FEEDBACK_OWN, MOTOR_FEEDBACK_OWN, MOTOR_FEEDFORWARD_ZERO, MOTOR_FEEDFORWARD_ZERO)

/* ================= 以下是给其他文件用的声明，不用改 ================= */

#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
#include "cdd/motor/dji_motor/dji_motor.h"
#define ROBOT_DECLARE_DJI_MOTOR(_name, ...)    extern dji_motor_t _name;
ROBOT_DJI_MOTOR_TABLE(ROBOT_DECLARE_DJI_MOTOR)
#endif

#ifdef CONFIG_CDD_MOTOR_DM_MOTOR
#include "cdd/motor/dm_motor/dm_motor.h"
#define ROBOT_DECLARE_DM_MOTOR(_name, ...)     extern dm_motor_t _name;
ROBOT_DM_MOTOR_TABLE(ROBOT_DECLARE_DM_MOTOR)
#endif

#endif
