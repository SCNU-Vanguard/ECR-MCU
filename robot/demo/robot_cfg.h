#ifndef ROBOT_CFG_H__
#define ROBOT_CFG_H__
/* demo 的构型：这台机器人有哪些电机、各挂在哪条 CAN 总线上、ID 是多少。
 * 框架不关心电机属于底盘还是云台，这里只列"电机 + 总线 + ID"，名字随便取（应用代码里直接用这个名字访问电机）。
 * 模块要不要编译、功能开不开，不在这里，在 robot/demo/demo_h7.config 的 CONFIG_XXX = y/n。
 *
 * 表是 X 宏：每行一个电机，行尾用反斜杠接下一行；表里没有电机 = 不注册任何电机。
 * 打开前确认总线上真有电机：经典 CAN 开了自动重传，发给无应答总线的帧会被不断重发，最终总线错误。
 * 每个电机会生成一个全局变量（名字就是表里的名字），robot_init.c 在初始化时按表绑定到 CAN 总线。 */
#include "bsp/include/bsp_ids.h"
#include "cdd/motor/motor.h"

/* ---- DJI 电机（M3508 / M2006 / GM6020）----
 * 列含义：
 *   名字 | 总线 | 型号 | ID(拨码) | 反向 | 编码器零点偏移 0..8191 | 离线超时 ms | GM6020 控制量(电压/电流) | 控制配置指针(NULL=不用软件控制)
 * 例（M3508 的 ID 1、2 挂在 CAN_BUS0；GM6020 的 ID 5 挂在 CAN_BUS1，电压模式）：
 *   #define ROBOT_DJI_MOTOR_TABLE(X)                                                                          \
 *       X(motor_a, CAN_BUS0, MOTOR_TYPE_M3508, 1, false, 0, 50, DJI_GM6020_VOLTAGE, &wheel_control)           \
 *       X(motor_b, CAN_BUS0, MOTOR_TYPE_M3508, 2, true,  0, 50, DJI_GM6020_VOLTAGE, &wheel_control)           \
 *       X(motor_c, CAN_BUS1, MOTOR_TYPE_GM6020, 5, false, 0, 50, DJI_GM6020_VOLTAGE, &steer_control)
 * 控制配置（motor_control_config_t）由使用者在别处定义成全局 const 变量，表里只引用它的地址。
 */
#define ROBOT_DJI_MOTOR_TABLE(X)                                                                                              \
    X(shoot_motor,   CAN_BUS0, MOTOR_TYPE_M3508, 1, false, 0, 50, DJI_GM6020_VOLTAGE, &shoot_motor_control)                  \
    X(chassis_motor, CAN_BUS1, MOTOR_TYPE_GM6020, 1, false, 0, 50, DJI_GM6020_VOLTAGE, &chassis_motor_control)

/* ---- DM 电机（DM4310 / DM8009P）----
 * 列含义：
 *   名字 | 总线 | 型号 | tx_id(电机 CAN ID) | master_id(反馈帧 ID) | 硬件模式 | 反向 | p_max rad | v_max rad/s | t_max N·m | 离线超时 ms
 *        | MIT 用法(原生/软件转矩) | Kp 0..500 | Kd 0..5 | 速度上限 rad/s | 电流上限 0..1 | 控制配置指针(NULL=不用软件控制)
 * 例（MIT 软件转矩：Kp=Kd=0，转矩由软件级联 PID 算，和 2026 步兵/标准一致）：
 *   #define ROBOT_DM_MOTOR_TABLE(X)                                                                                                                     \
 *       X(motor_d, CAN_BUS1, MOTOR_TYPE_DM4310, 1, 0x11, DM_MODE_MIT, false, 12.5f, 30.0f, 10.0f, 100, DM_MIT_SOFTWARE_TORQUE, 0.0f, 0.0f, 0.0f, 0.0f, &gimbal_control)
 */
#define ROBOT_DM_MOTOR_TABLE(X)                                                                                                                                            \
    X(gimbal_motor, CAN_BUS2, MOTOR_TYPE_DM4310, 1, 0x11, DM_MODE_MIT, false, 3.1415927f, 10.0f, 10.0f, 100, DM_MIT_SOFTWARE_TORQUE, 0.0f, 0.0f, 0.0f, 0.0f, &gimbal_motor_control)

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
