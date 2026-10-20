#ifndef MOTOR_CONFIG_H__
#define MOTOR_CONFIG_H__
/* demo 的电机表。只放数据；注册在 robot/motor_init.c，底盘尺寸在 chassis_config.h。
 * CAN_BUS0 = FDCAN1 发射，CAN_BUS1 = FDCAN2 底盘，CAN_BUS2 = FDCAN3 云台。
 * 底盘四个 m3508，ID 1–4 对应数学象限 1–4（右前、左前、左后、右后），速度环。
 * 云台 yaw 是 j4310（0x05 / 0x15），pitch 是 gm6020 ID 1，都是角度环。
 * 发射：m2006 ID 1 速度环；三个摩擦轮 m3508 ID 2/3/4 不建控制行，目标由 dji_motor_set_cmd 直接发给电调。
 * 不用软件控制的电机，电机表最后一列填 NULL。
 * 打开前确认总线上真有电机：经典 CAN 开了自动重传，发给无应答总线的帧会被不断重发。
 * 速度环 dji_motor 是 rad/s（转子）-> 电流/电压原始值，dm_motor 是 rad/s -> 转矩 N·m。
 * 注册时反馈用电机自己的，前馈为 0；云台在任务里改绑到 INS。
 * dm 的 p_max / v_max / t_max 必须和上位机一致。j4310：3.14 / 30 / 10。 */
#include "bsp/include/bsp_ids.h"
#include "cdd/motor/motor.h"

/* ================= dji_motor（m3508 / m2006 / gm6020） ================= */

#define ROBOT_DJI_MOTOR_CONFIG_TABLE(X) \
    X(chassis_m3508_0, CAN_BUS1, MOTOR_TYPE_M3508 , 1, false, 0, 50, GM6020_MOTOR_UNFIT  , &chassis_m3508_0_control) \
    X(chassis_m3508_1, CAN_BUS1, MOTOR_TYPE_M3508 , 2, false, 0, 50, GM6020_MOTOR_UNFIT  , &chassis_m3508_1_control) \
    X(chassis_m3508_2, CAN_BUS1, MOTOR_TYPE_M3508 , 3, false, 0, 50, GM6020_MOTOR_UNFIT  , &chassis_m3508_2_control) \
    X(chassis_m3508_3, CAN_BUS1, MOTOR_TYPE_M3508 , 4, false, 0, 50, GM6020_MOTOR_UNFIT  , &chassis_m3508_3_control) \
    X(gimbal_gm6020  , CAN_BUS2, MOTOR_TYPE_GM6020, 1, false, 0, 50, GM6020_MOTOR_VOLTAGE, &gimbal_gm6020_control  ) \
    X(shoot_m2006    , CAN_BUS0, MOTOR_TYPE_M2006 , 1, false, 0, 50, GM6020_MOTOR_UNFIT  , &shoot_m2006_control    ) \
    X(shoot_fric_0   , CAN_BUS0, MOTOR_TYPE_M3508 , 2, false, 0, 50, GM6020_MOTOR_UNFIT  , NULL                    ) \
    X(shoot_fric_1   , CAN_BUS0, MOTOR_TYPE_M3508 , 3, false, 0, 50, GM6020_MOTOR_UNFIT  , NULL                    ) \
    X(shoot_fric_2   , CAN_BUS0, MOTOR_TYPE_M3508 , 4, false, 0, 50, GM6020_MOTOR_UNFIT  , NULL                    )
/* 名字           , 总线    , 型号             , ID, 反向 , 零点, 超时, gm6020控制量      , 控制参数 */

#define ROBOT_DJI_MOTOR_CONTROL_TABLE(X) \
    X(chassis_m3508_0, MOTOR_CONTROL_SPEED, 0.0f , 0.0f, 0.0f, 0.0f, 0.0f , 0.0f , 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(chassis_m3508_1, MOTOR_CONTROL_SPEED, 0.0f , 0.0f, 0.0f, 0.0f, 0.0f , 0.0f , 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(chassis_m3508_2, MOTOR_CONTROL_SPEED, 0.0f , 0.0f, 0.0f, 0.0f, 0.0f , 0.0f , 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(chassis_m3508_3, MOTOR_CONTROL_SPEED, 0.0f , 0.0f, 0.0f, 0.0f, 0.0f , 0.0f , 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(shoot_m2006    , MOTOR_CONTROL_SPEED, 0.0f , 0.0f, 0.0f, 0.0f, 0.0f , 0.0f , 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(gimbal_gm6020  , MOTOR_CONTROL_ANGLE, 10.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f, 0.0f, 0.0f, 500.0f, 0.0f, 0.0f, 0.0f, 15000.0f, 15000.0f, 0.0f, 0.0f)
/* 名字           , 模式               , 角kp , 角ki, 角kd, 角kf, 角输出, 角积分, 角前馈, 角死区, 速kp  , 速ki, 速kd, 速kf, 速输出  , 速积分  , 速前馈, 速死区 */

/* ================= dm_motor（j4310） ================= */

#define ROBOT_DM_MOTOR_CONFIG_TABLE(X) \
    X(gimbal_j4310, CAN_BUS2, MOTOR_TYPE_J4310, 0x05, 0x15, DM_MOTOR_MODE_TORQUE, false, 3.14f, 30.0f, 10.0f, 100, 0.0f, 0.0f, 0.0f, 0.0f, &gimbal_j4310_control)
/* 名字        , 总线    , 型号           , tx_id, master, 控制模式           , 反向 , p_max, v_max, t_max, 超时, Kp  , Kd  , 速度上限, 电流上限, 控制参数 */

#define ROBOT_DM_MOTOR_CONTROL_TABLE(X) \
    X(gimbal_j4310, MOTOR_CONTROL_ANGLE, 15.0f, 0.0f, 0.0f, 0.0f, 5.0f, 5.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 5.0f, 5.0f, 0.0f, 0.0f)
/* 名字        , 模式               , 角kp , 角ki, 角kd, 角kf, 角输出, 角积分, 角前馈, 角死区, 速kp, 速ki, 速kd, 速kf, 速输出, 速积分, 速前馈, 速死区 */

#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
#include "cdd/motor/dji_motor/dji_motor.h"
#define ROBOT_DECLARE_DJI_MOTOR(_name, ...)    extern dji_motor_t _name;
ROBOT_DJI_MOTOR_CONFIG_TABLE(ROBOT_DECLARE_DJI_MOTOR)
#endif

#ifdef CONFIG_CDD_MOTOR_DM_MOTOR
#include "cdd/motor/dm_motor/dm_motor.h"
#define ROBOT_DECLARE_DM_MOTOR(_name, ...)     extern dm_motor_t _name;
ROBOT_DM_MOTOR_CONFIG_TABLE(ROBOT_DECLARE_DM_MOTOR)
#endif

#endif
