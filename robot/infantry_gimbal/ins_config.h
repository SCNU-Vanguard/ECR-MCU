#ifndef INS_CONFIG_H__
#define INS_CONFIG_H__
#include "component/ahrs/ahrs.h"

/* 本兵种惯导。算法、数据源、标定、加热都在这里，改完重新编译烧录。
 * 算法对应的 CONFIG_COMPONENT_AHRS_* 要在 robot.config 里打开，否则 ahrs_start 返回 ERR_NODEV。 */
#define INS_ALGORITHM                       AHRS_ALGORITHM_VQF

/* 数据源。BMI088：SPI，配合板上加热。HI05R3：经典 CAN、1 Mbit/s、J1939，出厂节点 8。
 * HI05R3_BUS 选总线：CAN_BUS1 = FDCAN1，CAN_BUS2 = FDCAN2，CAN_BUS3 = FDCAN3。
 * 模块输出周期要设成 1000000/HI05R3_RATE_HZ 微秒（200 Hz 就是 5 ms），并打开加速度、角速度、横滚/俯仰、航向。
 * HI05R3_USE_DEVICE_ATTITUDE 为 0 时仍用 INS_ALGORITHM 解算；为 1 时直接采用模块欧拉角。 */
#define INS_IMU_BMI088                      0
#define INS_IMU_HI05R3                      1
#define INS_IMU                             INS_IMU_BMI088
#define HI05R3_BUS                          CAN_BUS1
#define HI05R3_NODE_ID                      8u
#define HI05R3_RATE_HZ                      200u
#define HI05R3_USE_DEVICE_ATTITUDE          1

/* 标定表，一行一个轴（X、Y、Z 顺序），列含义：
 *   加速度计 scale（无量纲） | 加速度计 offset（m/s²） | 陀螺仪 offset（rad/s）
 * 加速度 = 灵敏度 * raw * scale - scale * offset；角速度 = 灵敏度 * raw - offset。
 * 陀螺零偏 2026-10-04，约 38.5 °C 下静止标定。换板子或温度差别大时重新跑 ins_cali，把打印的表贴回这里。 */
#define BMI088_CALIBRATION_TABLE(X) \
    X(1.0108006001f, -0.0241884422f, 0.0031073433f) \
    X(1.0080075264f, 0.0630175471f, -0.0009981639f) \
    X(1.0091868639f, 0.0795006827f, -0.0011280888f)

/* 恒温加热（PB1 / TIM3_CH4，500 Hz PWM）。heater_step 按 10 Hz 调用（和 BMI088 读温度的频率一致），ki、kd 按这个周期整定。
 * 预热：仅在冷启动温度偏低时用 25% 占空比，到达目标 - 余量后转入恒温。
 * 恒温：位置式 PID，进入恒温时把积分项预置到一个估计的维持占空比。
 * 恒温段的 PID 参数没有整定过，维持温度需要的占空比靠实测调整。 */
#define HEATER_TARGET_C                 45.0f       /* 接近陀螺零偏标定温度 */
#define HEATER_SHUTDOWN_C               50.0f       /* 独立过温停热保护 */

#define HEATER_PREHEAT_DUTY             0.25f
#define HEATER_PREHEAT_MARGIN_C         5.0f

#define HEATER_KP                       0.05f       /* 单位：占空比 / °C */
#define HEATER_KI                       0.1f        /* 每个调用周期累加 */
#define HEATER_KD                       0.0f
#define HEATER_INTEGRAL_LIMIT           0.02f
#define HEATER_INTEGRAL_PRESET          0.01f       /* 转入恒温时的积分初值，不能超过 INTEGRAL_LIMIT */
#define HEATER_MAX_DUTY                 0.02f       /* 恒温段最大占空比 2% ， 如果板子是完全使用TPU外壳就直接换 1% 占空比，但是需要同步更改IMU的加热积分上限 */
#define HEATER_DEAD_BAND_C              0.0f
#endif
