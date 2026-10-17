#ifndef ROBOT_CFG_H__
#define ROBOT_CFG_H__
/* demo 的构型：这台机器人有哪些电机、各挂在哪条 CAN 总线上、ID 是多少、软件控制的 PID 是多少。
 * 这个文件只放数据，不放逻辑；定义变量和绑定的代码都在 robot_init.c，改机器人配置只需要改这里。
 * 模块要不要编译、功能开不开，不在这里，在 robot/demo/demo.config 的 CONFIG_XXX = y/n。
 *
 * 这次是三路 CAN 的多电机测试。每路都挂同样四台，ID 也一样：
 *   CAN_BUS0 = FDCAN1，shoot 任务；CAN_BUS1 = FDCAN2，chassis 任务；CAN_BUS2 = FDCAN3，gimbal 任务。
 *   M3508 ID 4，GM6020 ID 1，DM3519 CAN ID 0x09、Master ID 0x29，DM4310 CAN ID 5、Master ID 0x15。
 * M3508 和 DM3519 走速度环，GM6020 和 DM4310 走角度环串速度环。
 *
 * 电机表决定挂在哪、怎么通信。控制表一台电机一行，行首名字和电机表相同，电机表最后一列引用它。
 * 不用软件控制的电机不用写控制表，电机表最后一列填 NULL。
 * 表里没有电机 = 不注册任何电机。每个电机会生成一个全局变量，名字就是表里的名字。
 * 打开前确认总线上真有电机：经典 CAN 开了自动重传，发给无应答总线的帧会被不断重发，最终总线错误。
 *
 * 模式在控制表里写死：SPEED 只用速度环，ANGLE 是角度环串速度环，TORQUE 两个环都不跑。
 * 角度环单位 rad -> rad/s，位置式；速度环 DJI 是 rad/s -> 电流/电压原始值，DM 是 rad/s -> 转矩 N·m，增量式。
 * 只用速度环时角度环填 0。反馈默认 MOTOR_FEEDBACK_OWN，前馈默认 MOTOR_FEEDFORWARD_ZERO。
 * 这些是没有实测过的保守初值，现场用 shell 的 xxx_pid 命令调。
 * DM 的 p_max / v_max / t_max 必须和上位机 PMAX / VMAX / TMAX 一致，否则反馈和控制帧都会编解码错。
 * 3519：1000 / 40 / 10。4310：3.14 / 30 / 10。3519 的位置映射是电机侧，速度和转矩是输出轴。 */
#include "bsp/include/bsp_ids.h"
#include "cdd/motor/motor.h"

/* ================= DJI 电机（M3508 / M2006 / GM6020） ================= */

#define ROBOT_DJI_MOTOR_TABLE(X)                                                                                  \
    X(shoot_m3508,    CAN_BUS0,  MOTOR_TYPE_M3508,   4,  false,  0,  50,  DJI_GM6020_VOLTAGE,  &shoot_m3508_control   )  \
    X(shoot_gm6020,   CAN_BUS0,  MOTOR_TYPE_GM6020,  3,  false,  0,  50,  DJI_GM6020_VOLTAGE,  &shoot_gm6020_control  )  \
    X(chassis_m3508,  CAN_BUS1,  MOTOR_TYPE_M3508,   4,  false,  0,  50,  DJI_GM6020_VOLTAGE,  &chassis_m3508_control )  \
    X(chassis_gm6020, CAN_BUS1,  MOTOR_TYPE_GM6020,  3,  false,  0,  50,  DJI_GM6020_VOLTAGE,  &chassis_gm6020_control)  \
    X(gimbal_m3508,   CAN_BUS2,  MOTOR_TYPE_M3508,   4,  false,  0,  50,  DJI_GM6020_VOLTAGE,  &gimbal_m3508_control  )  \
    X(gimbal_gm6020,  CAN_BUS2,  MOTOR_TYPE_GM6020,  3,  false,  0,  50,  DJI_GM6020_VOLTAGE,  &gimbal_gm6020_control )
/*    名字         ,  总线    ,  型号             ,  ID,  反向 ,  零点, 超时,  控制量            ,  控制配置      */

#define ROBOT_DJI_CONTROL_TABLE(X)                                                                                                                                                                                                          \
    X(shoot_m3508,    MOTOR_CONTROL_SPEED,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f, 100.0f,  0.0f,  0.0f,  0.0f, 10000.0f, 10000.0f,  0.0f,  0.0f,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDFORWARD_ZERO,  MOTOR_FEEDFORWARD_ZERO) \
    X(shoot_gm6020,   MOTOR_CONTROL_ANGLE, 10.0f,  0.0f,  0.0f,  0.0f, 20.0f, 20.0f,  0.0f,  0.0f, 500.0f,  0.0f,  0.0f,  0.0f, 15000.0f, 15000.0f,  0.0f,  0.0f,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDFORWARD_ZERO,  MOTOR_FEEDFORWARD_ZERO) \
    X(chassis_m3508,  MOTOR_CONTROL_SPEED,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f, 100.0f,  0.0f,  0.0f,  0.0f, 10000.0f, 10000.0f,  0.0f,  0.0f,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDFORWARD_ZERO,  MOTOR_FEEDFORWARD_ZERO) \
    X(chassis_gm6020, MOTOR_CONTROL_ANGLE, 10.0f,  0.0f,  0.0f,  0.0f, 20.0f, 20.0f,  0.0f,  0.0f, 500.0f,  0.0f,  0.0f,  0.0f, 15000.0f, 15000.0f,  0.0f,  0.0f,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDFORWARD_ZERO,  MOTOR_FEEDFORWARD_ZERO) \
    X(gimbal_m3508,   MOTOR_CONTROL_SPEED,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f, 100.0f,  0.0f,  0.0f,  0.0f, 10000.0f, 10000.0f,  0.0f,  0.0f,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDFORWARD_ZERO,  MOTOR_FEEDFORWARD_ZERO) \
    X(gimbal_gm6020,  MOTOR_CONTROL_ANGLE, 10.0f,  0.0f,  0.0f,  0.0f, 20.0f, 20.0f,  0.0f,  0.0f, 500.0f,  0.0f,  0.0f,  0.0f, 15000.0f, 15000.0f,  0.0f,  0.0f,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDFORWARD_ZERO,  MOTOR_FEEDFORWARD_ZERO)
/*    名字  ,  模式                ,  角kp,  角ki,  角kd,  角kf,  角输出限幅, 角积分限幅, 角前馈限幅, 角死区, 速kp, 速ki, 速kd, 速kf,  速输出限幅, 速积分限幅, 速前馈限幅, 速死区, 角度反馈          ,  速度反馈          ,  速度前馈              ,  转矩前馈               */

/* ================= DM 电机（DM4310 / DM8009P / DM3519） ================= */

#define ROBOT_DM_MOTOR_TABLE(X)                                                                                                                                                                                                   \
    X(shoot_dm3519,    CAN_BUS0,  MOTOR_TYPE_DM3519,  0x09,  0x29,  DM_MODE_MIT,  false,  1000.0f,  40.0f,  10.0f,  100,  DM_MIT_SOFTWARE_TORQUE,  0.0f,  0.0f,  0.0f,  0.0f,  &shoot_dm3519_control  ) \
    X(shoot_dm4310,    CAN_BUS0,  MOTOR_TYPE_DM4310,  0x05,  0x15,  DM_MODE_MIT,  false,    3.14f,  30.0f,  10.0f,  100,  DM_MIT_SOFTWARE_TORQUE,  0.0f,  0.0f,  0.0f,  0.0f,  &shoot_dm4310_control  ) \
    X(chassis_dm3519,  CAN_BUS1,  MOTOR_TYPE_DM3519,  0x09,  0x29,  DM_MODE_MIT,  false,  1000.0f,  40.0f,  10.0f,  100,  DM_MIT_SOFTWARE_TORQUE,  0.0f,  0.0f,  0.0f,  0.0f,  &chassis_dm3519_control) \
    X(chassis_dm4310,  CAN_BUS1,  MOTOR_TYPE_DM4310,  0x05,  0x15,  DM_MODE_MIT,  false,    3.14f,  30.0f,  10.0f,  100,  DM_MIT_SOFTWARE_TORQUE,  0.0f,  0.0f,  0.0f,  0.0f,  &chassis_dm4310_control) \
    X(gimbal_dm3519,   CAN_BUS2,  MOTOR_TYPE_DM3519,  0x09,  0x29,  DM_MODE_MIT,  false,  1000.0f,  40.0f,  10.0f,  100,  DM_MIT_SOFTWARE_TORQUE,  0.0f,  0.0f,  0.0f,  0.0f,  &gimbal_dm3519_control ) \
    X(gimbal_dm4310,   CAN_BUS2,  MOTOR_TYPE_DM4310,  0x05,  0x15,  DM_MODE_MIT,  false,    3.14f,  30.0f,  10.0f,  100,  DM_MIT_SOFTWARE_TORQUE,  0.0f,  0.0f,  0.0f,  0.0f,  &gimbal_dm4310_control )
/*    名字          ,  总线    ,  型号              ,  tx_id, master_id, 硬件模式  ,  反向 ,  p_max     ,  v_max,  t_max, 超时,  MIT用法               ,  Kp  ,  Kd  , 速度上限, 电流上限, 控制配置       */

#define ROBOT_DM_CONTROL_TABLE(X)                                                                                                                                                                                                       \
    X(shoot_dm3519,    MOTOR_CONTROL_SPEED,   0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.04f,  0.0f,  0.0f,  0.0f,  0.3f,  0.3f,  0.0f,  0.0f,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDFORWARD_ZERO,  MOTOR_FEEDFORWARD_ZERO) \
    X(shoot_dm4310,    MOTOR_CONTROL_ANGLE,  15.0f,  0.0f,  0.0f,  0.0f,  5.0f,  5.0f,  0.0f,  0.0f,   1.0f,  0.0f,  0.0f,  0.0f,  5.0f,  5.0f,  0.0f,  0.0f,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDFORWARD_ZERO,  MOTOR_FEEDFORWARD_ZERO) \
    X(chassis_dm3519,  MOTOR_CONTROL_SPEED,   0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.04f,  0.0f,  0.0f,  0.0f,  0.3f,  0.3f,  0.0f,  0.0f,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDFORWARD_ZERO,  MOTOR_FEEDFORWARD_ZERO) \
    X(chassis_dm4310,  MOTOR_CONTROL_ANGLE,  15.0f,  0.0f,  0.0f,  0.0f,  5.0f,  5.0f,  0.0f,  0.0f,   1.0f,  0.0f,  0.0f,  0.0f,  5.0f,  5.0f,  0.0f,  0.0f,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDFORWARD_ZERO,  MOTOR_FEEDFORWARD_ZERO) \
    X(gimbal_dm3519,   MOTOR_CONTROL_SPEED,   0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.04f,  0.0f,  0.0f,  0.0f,  0.3f,  0.3f,  0.0f,  0.0f,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDFORWARD_ZERO,  MOTOR_FEEDFORWARD_ZERO) \
    X(gimbal_dm4310,   MOTOR_CONTROL_ANGLE,  15.0f,  0.0f,  0.0f,  0.0f,  5.0f,  5.0f,  0.0f,  0.0f,   1.0f,  0.0f,  0.0f,  0.0f,  5.0f,  5.0f,  0.0f,  0.0f,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDBACK_OWN,  MOTOR_FEEDFORWARD_ZERO,  MOTOR_FEEDFORWARD_ZERO)
/*            名字 ,   模式                , 角kp,  角ki,  角kd,  角kf,角输出限幅,角积分限幅,角前馈限幅, 角死区, 速kp ,  速ki  ,  速kd,  速kf,速输出限幅,速积分限幅,速前馈限幅, 速死区, 角度反馈          ,  速度反馈          ,  速度前馈              ,  转矩前馈               */

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
