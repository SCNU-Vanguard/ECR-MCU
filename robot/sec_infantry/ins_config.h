#ifndef INS_CONFIG_H__
#define INS_CONFIG_H__
#include "component/ahrs/ahrs.h"

/* 本兵种惯导。算法、标定、加热都在这里，改完重新编译烧录。
 * 算法对应的 CONFIG_COMPONENT_AHRS_* 要在 robot.config 里打开，否则 ahrs_start 返回 ERR_NODEV。 */
#define INS_ALGORITHM                       AHRS_ALGORITHM_VQF

/* 标定表，一行一个轴（X、Y、Z 顺序），列含义：
 *   加速度计 scale（无量纲） | 加速度计 offset（m/s²） | 陀螺仪 offset（rad/s）
 * 加速度 = 灵敏度 * raw * scale - scale * offset；角速度 = 灵敏度 * raw - offset。
 * 陀螺零偏 2026-10-04，约 38.5 °C 下静止标定。换板子或温度差别大时重新跑 ins_cali，把打印的表贴回这里。 */
#define BMI088_CALIBRATION_TABLE(X) \
    X(1.0108006001f, -0.0241884422f, 0.0031073433f) \
    X(1.0080075264f, 0.0630175471f, -0.0009981639f) \
    X(1.0091868639f, 0.0795006827f, -0.0011280888f)

/* 恒温加热（PB1 / TIM3_CH4，500 Hz PWM）。heater_step 按 10 Hz 调用（和 BMI088 读温度的频率一致），ki、kd 按这个周期整定。
 * 预热：大占空比持续加热，直到温度进入 [目标 - 预热余量] 并累计保持一段时间。
 * 恒温：位置式 PID，进入恒温时把积分项预置到一个估计的维持占空比。
 * 恒温段的 PID 参数没有整定过，维持温度需要的占空比靠实测调整。 */
#define IMU_HEATER_TARGET_C                 40.0f       /* 目标温度，陀螺零偏在约 40 °C 下标定 */
#define IMU_HEATER_SHUTDOWN_C               45.0f       /* 超过就停止加热（硬保护） */

#define IMU_HEATER_PREHEAT_DUTY             0.75f       /* 预热占空比（0..1） */
#define IMU_HEATER_PREHEAT_MARGIN_C         5.0f        /* 温度达到 目标 - 余量 开始计时 */
#define IMU_HEATER_PREHEAT_HOLD_CYCLES      20u         /* 累计满这么多个周期（10 Hz 下 2 s）后转入恒温 */

#define IMU_HEATER_KP                       0.1f        /* 单位：占空比 / °C */
#define IMU_HEATER_KI                       0.05f       /* 每个调用周期累加 */
#define IMU_HEATER_KD                       0.0f
#define IMU_HEATER_INTEGRAL_LIMIT           0.25f
#define IMU_HEATER_INTEGRAL_PRESET          0.25f       /* 转入恒温时的积分初值，不能超过 INTEGRAL_LIMIT */
#define IMU_HEATER_MAX_DUTY                 0.5f        /* 恒温段最大占空比 */
#define IMU_HEATER_DEAD_BAND_C              0.0f
#endif
