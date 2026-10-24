#ifndef MOTOR_CONFIG_H__
#define MOTOR_CONFIG_H__
/* 底盘板电机。CAN_BUS1 = FDCAN1，CAN_BUS2 = FDCAN2，CAN_BUS3 = FDCAN3。
 * CAN1：四个全向轮 m3508，ID 1–4 对应数学象限 1–4（右前、左前、左后、右后），速度环。
 * CAN2：yaw，j4310。控制 ID 1，反馈 Master ID 0x11（沿用原来 tx 和 master 相差 0x10）。
 *       角度当前值和目标由云台 IMU 经板间传来。速度环仍用电机自己的转速，任务里再绑自旋前馈。
 * CAN3：拨弹 m2006，ID 1，速度环。只给 0 或恒定目标。
 * 超电也在 CAN2，标识符是 0x05A / 0x06A，和 yaw 不重叠。
 * 瓴控暂不挂。驱动和注册框架留着，robot.config 里关掉，表不出现。 */
/* ================= dji_motor（m3508 / m2006） ================= */

#define ROBOT_DJI_MOTOR_CONFIG_TABLE(X) \
    X(chassis_m3508_0, CAN_BUS1, MOTOR_TYPE_M3508, 1, false, 0, 50, GM6020_MOTOR_UNFIT, &chassis_m3508_0_control) \
    X(chassis_m3508_1, CAN_BUS1, MOTOR_TYPE_M3508, 2, false, 0, 50, GM6020_MOTOR_UNFIT, &chassis_m3508_1_control) \
    X(chassis_m3508_2, CAN_BUS1, MOTOR_TYPE_M3508, 3, false, 0, 50, GM6020_MOTOR_UNFIT, &chassis_m3508_2_control) \
    X(chassis_m3508_3, CAN_BUS1, MOTOR_TYPE_M3508, 4, false, 0, 50, GM6020_MOTOR_UNFIT, &chassis_m3508_3_control) \
    X(shoot_m2006    , CAN_BUS3, MOTOR_TYPE_M2006, 1, false, 0, 50, GM6020_MOTOR_UNFIT, &shoot_m2006_control    )
/* 名字           , 总线    , 型号            , ID, 反向 , 零点, 超时, gm6020控制量    , 控制参数 */

#define ROBOT_DJI_MOTOR_CONTROL_TABLE(X) \
    X(chassis_m3508_0, MOTOR_CONTROL_SPEED, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(chassis_m3508_1, MOTOR_CONTROL_SPEED, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(chassis_m3508_2, MOTOR_CONTROL_SPEED, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(chassis_m3508_3, MOTOR_CONTROL_SPEED, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(shoot_m2006    , MOTOR_CONTROL_SPEED, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f)
/* 名字           , 模式               , 角kp, 角ki, 角kd, 角kf, 角输出, 角积分, 角前馈, 角死区, 速kp  , 速ki, 速kd, 速kf, 速输出  , 速积分  , 速前馈, 速死区 */

/* ================= dm_motor（j4310） ================= */

#define ROBOT_DM_MOTOR_CONFIG_TABLE(X) \
    X(yaw_j4310, CAN_BUS2, MOTOR_TYPE_J4310, 1, 0x11, DM_MOTOR_MODE_TORQUE, false, 3.14f, 30.0f, 10.0f, 100, 0.0f, 0.0f, 0.0f, 0.0f, &yaw_j4310_control)
/* 名字     , 总线    , 型号           , tx_id, master, 控制模式           , 反向 , p_max, v_max, t_max, 超时, Kp , Kd , 速度上限, 电流上限, 控制参数 */

#define ROBOT_DM_MOTOR_CONTROL_TABLE(X) \
    X(yaw_j4310, MOTOR_CONTROL_ANGLE, 15.0f, 0.0f, 0.0f, 0.0f, 5.0f, 5.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 5.0f, 5.0f, 0.0f, 0.0f)
/* 名字     , 模式               , 角kp , 角ki, 角kd, 角kf, 角输出, 角积分, 角前馈, 角死区, 速kp, 速ki, 速kd, 速kf, 速输出, 速积分, 速前馈, 速死区 */
#endif
