#ifndef MOTOR_CONFIG_H__
#define MOTOR_CONFIG_H__
/* 云台板电机。CAN_BUS1 = FDCAN1，CAN_BUS2 = FDCAN2，CAN_BUS3 = FDCAN3。
 * CAN1：pitch，gm6020，ID 1，角度环，反馈在任务里改绑到本板 IMU。
 * CAN2：三个摩擦轮 m3508，ID 1、2、3，不建控制行，任务里 dji_motor_set_cmd 直接给原始值。全正向。
 * 本板没有达妙和瓴控。不用的那类表不出现；对应驱动在 robot.config 里关掉。 */
/* ================= dji_motor（m3508 / gm6020） ================= */

#define ROBOT_DJI_MOTOR_CONFIG_TABLE(X) \
    X(pitch_gm6020, CAN_BUS1, MOTOR_TYPE_GM6020, 1, false, 0, 50, GM6020_MOTOR_VOLTAGE, &pitch_gm6020_control) \
    X(shoot_fric_0, CAN_BUS2, MOTOR_TYPE_M3508 , 1, false, 0, 50, GM6020_MOTOR_UNFIT  , NULL                 ) \
    X(shoot_fric_1, CAN_BUS2, MOTOR_TYPE_M3508 , 2, false, 0, 50, GM6020_MOTOR_UNFIT  , NULL                 ) \
    X(shoot_fric_2, CAN_BUS2, MOTOR_TYPE_M3508 , 3, false, 0, 50, GM6020_MOTOR_UNFIT  , NULL                 )
/* 名字         , 总线    , 型号             , ID, 反向 , 零点, 超时, gm6020控制量      , 控制参数 */

#define ROBOT_DJI_MOTOR_CONTROL_TABLE(X) \
    X(pitch_gm6020, MOTOR_CONTROL_ANGLE, 10.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f, 0.0f, 0.0f, 500.0f, 0.0f, 0.0f, 0.0f, 15000.0f, 15000.0f, 0.0f, 0.0f)
/* 名字         , 模式               , 角kp , 角ki, 角kd, 角kf, 角输出, 角积分, 角前馈, 角死区, 速kp  , 速ki, 速kd, 速kf, 速输出  , 速积分  , 速前馈, 速死区 */
#endif
