#ifndef INS_CONFIG_H__
#define INS_CONFIG_H__
#include "cdd/imu/ins/ins.h"

/* 一次只选一个 IMU，另一段不编译。算法和对应驱动的 CONFIG_* 要打开。 */
#define INS_ALGORITHM                       AHRS_ALGORITHM_VQF
#define INS_IMU                             INS_IMU_BMI088

/* 1：静止台架学习零偏。上车前改为 0。 */
#define INS_ASSUME_STATIONARY               0U

#if INS_IMU == INS_IMU_BMI088

/* X/Y/Z：加速度 scale、加速度 offset（m/s²）、陀螺 offset（rad/s）。45.352587 °C 标定。 */
#define BMI088_CALIBRATION_TABLE(X) \
    X(1.0082575754f, -0.0773002426f,  0.004310322192f) \
    X(1.0059449421f,  0.0056996940f, -0.007818833371f) \
    X(1.0075691343f,  0.0187196156f,  0.001523378717f)

/* PB1 / TIM3_CH4，10 Hz。打开 CONFIG_CDD_IMU_BMI088_HEATER 才生效。TPU 全包外壳把 MAX_DUTY 改为 0.01，并降低积分上限。 */
#define BMI088_HEATER_TARGET_C              45.0f
#define BMI088_HEATER_SHUTDOWN_C            50.0f
#define BMI088_HEATER_PREHEAT_DUTY          0.25f
#define BMI088_HEATER_PREHEAT_MARGIN_C      5.0f
#define BMI088_HEATER_KP                    0.05f       /* 占空比 / °C */
#define BMI088_HEATER_KI                    0.1f        /* 每个调用周期累加 */
#define BMI088_HEATER_KD                    0.0f
#define BMI088_HEATER_INTEGRAL_LIMIT        0.02f
#define BMI088_HEATER_INTEGRAL_PRESET       0.01f       /* 不超过 INTEGRAL_LIMIT */
#define BMI088_HEATER_MAX_DUTY              0.02f
#define BMI088_HEATER_DEAD_BAND_C           0.0f

_Static_assert(BMI088_HEATER_INTEGRAL_PRESET <= BMI088_HEATER_INTEGRAL_LIMIT,
               "BMI088_HEATER_INTEGRAL_PRESET must not exceed BMI088_HEATER_INTEGRAL_LIMIT");

#elif INS_IMU == INS_IMU_HI05R3

/* CAN_BUS1/2/3 = FDCAN1/2/3。周期 1000000/HI05R3_RATE_HZ 微秒，并打开加速度、角速度、四元数。
 * HI05R3_ATTITUDE_AHRS 用 INS_ALGORITHM，HI05R3_ATTITUDE_DEVICE 用模块四元数。 */
#define HI05R3_BUS                          CAN_BUS1
#define HI05R3_NODE_ID                      8u
#define HI05R3_RATE_HZ                      200u
#define HI05R3_ATTITUDE                     HI05R3_ATTITUDE_DEVICE

#else
#error "INS_IMU must be INS_IMU_BMI088 or INS_IMU_HI05R3"
#endif
#endif
