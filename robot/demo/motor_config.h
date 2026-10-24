#ifndef MOTOR_CONFIG_H__
#define MOTOR_CONFIG_H__
/* 单板电机。型号、PID 和双板相同，只是三条总线合在这一块板上。
 * CAN1：四个全向轮 m3508，ID 1–4 对应右前、左前、左后、右后，速度环。
 * CAN2：pitch gm6020 ID 1；yaw j4310，控制 ID 1，反馈 Master ID 0x11。超电也在这条总线。
 * CAN3：三个摩擦轮 m3508 ID 1、2、3，不建控制行，任务里直接给原始值，全正向。
 *       拨弹 m2006 接着排，ID 4，和摩擦轮同在 0x200 控制帧里。速度环，只给 0 或恒定目标。
 * 瓴控暂不挂。驱动和注册框架留着，robot.config 里关掉，表不出现。 */
/* ================= dji_motor（m3508 / m2006 / gm6020） ================= */

#define ROBOT_DJI_MOTOR_CONFIG_TABLE(X) \
    X(chassis_m3508_0, CAN_BUS1, MOTOR_TYPE_M3508 , 1, false, 0, 50, GM6020_MOTOR_UNFIT  , &chassis_m3508_0_control) \
    X(chassis_m3508_1, CAN_BUS1, MOTOR_TYPE_M3508 , 2, false, 0, 50, GM6020_MOTOR_UNFIT  , &chassis_m3508_1_control) \
    X(chassis_m3508_2, CAN_BUS1, MOTOR_TYPE_M3508 , 3, false, 0, 50, GM6020_MOTOR_UNFIT  , &chassis_m3508_2_control) \
    X(chassis_m3508_3, CAN_BUS1, MOTOR_TYPE_M3508 , 4, false, 0, 50, GM6020_MOTOR_UNFIT  , &chassis_m3508_3_control) \
    X(pitch_gm6020   , CAN_BUS2, MOTOR_TYPE_GM6020, 1, false, 0, 50, GM6020_MOTOR_VOLTAGE, &pitch_gm6020_control   ) \
    X(shoot_fric_0   , CAN_BUS3, MOTOR_TYPE_M3508 , 1, false, 0, 50, GM6020_MOTOR_UNFIT  , NULL                    ) \
    X(shoot_fric_1   , CAN_BUS3, MOTOR_TYPE_M3508 , 2, false, 0, 50, GM6020_MOTOR_UNFIT  , NULL                    ) \
    X(shoot_fric_2   , CAN_BUS3, MOTOR_TYPE_M3508 , 3, false, 0, 50, GM6020_MOTOR_UNFIT  , NULL                    ) \
    X(shoot_m2006    , CAN_BUS3, MOTOR_TYPE_M2006 , 4, false, 0, 50, GM6020_MOTOR_UNFIT  , &shoot_m2006_control    )
/* 名字           , 总线    , 型号             , ID, 反向 , 零点, 超时, gm6020控制量      , 控制参数 */

#define ROBOT_DJI_MOTOR_CONTROL_TABLE(X) \
    X(chassis_m3508_0, MOTOR_CONTROL_SPEED, 0.0f , 0.0f, 0.0f, 0.0f, 0.0f , 0.0f , 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(chassis_m3508_1, MOTOR_CONTROL_SPEED, 0.0f , 0.0f, 0.0f, 0.0f, 0.0f , 0.0f , 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(chassis_m3508_2, MOTOR_CONTROL_SPEED, 0.0f , 0.0f, 0.0f, 0.0f, 0.0f , 0.0f , 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(chassis_m3508_3, MOTOR_CONTROL_SPEED, 0.0f , 0.0f, 0.0f, 0.0f, 0.0f , 0.0f , 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(shoot_m2006    , MOTOR_CONTROL_SPEED, 0.0f , 0.0f, 0.0f, 0.0f, 0.0f , 0.0f , 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.0f, 0.0f) \
    X(pitch_gm6020   , MOTOR_CONTROL_ANGLE, 10.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f, 0.0f, 0.0f, 500.0f, 0.0f, 0.0f, 0.0f, 15000.0f, 15000.0f, 0.0f, 0.0f)
/* 名字           , 模式               , 角kp , 角ki, 角kd, 角kf, 角输出, 角积分, 角前馈, 角死区, 速kp  , 速ki, 速kd, 速kf, 速输出  , 速积分  , 速前馈, 速死区 */

/* ================= dm_motor（j4310） ================= */

#define ROBOT_DM_MOTOR_CONFIG_TABLE(X) \
    X(yaw_j4310, CAN_BUS2, MOTOR_TYPE_J4310, 1, 0x11, DM_MOTOR_MODE_TORQUE, false, 3.14f, 30.0f, 10.0f, 100, 0.0f, 0.0f, 0.0f, 0.0f, &yaw_j4310_control)
/* 名字     , 总线    , 型号           , tx_id, master, 控制模式           , 反向 , p_max, v_max, t_max, 超时, Kp , Kd , 速度上限, 电流上限, 控制参数 */

#define ROBOT_DM_MOTOR_CONTROL_TABLE(X) \
    X(yaw_j4310, MOTOR_CONTROL_ANGLE, 15.0f, 0.0f, 0.0f, 0.0f, 5.0f, 5.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 5.0f, 5.0f, 0.0f, 0.0f)
/* 名字     , 模式               , 角kp , 角ki, 角kd, 角kf, 角输出, 角积分, 角前馈, 角死区, 速kp, 速ki, 速kd, 速kf, 速输出, 速积分, 速前馈, 速死区 */
#endif
