#ifndef CDD_IMU_HEATER_CONFIG_H__
#define CDD_IMU_HEATER_CONFIG_H__

/* IMU 恒温加热参数（PB1 / TIM3_CH4，500 Hz PWM）。imu_heater_step 按 10 Hz 调用（和 BMI088 驱动读温度的频率一致），ki、kd 按这个周期整定。
 * 控制分两段（照 standard 工程里被注释掉的 BMI088_Temp_Control）：
 *   预热：大占空比持续加热，直到温度进入 [目标 - 预热余量] 并累计保持一段时间
 *   恒温：位置式 PID，进入恒温时把积分项预置到一个估计的维持占空比，加速收敛
 * 恒温段的 PID 参数没有整定过，维持温度需要的占空比靠实测调整。 */
#define IMU_HEATER_TARGET_C                 40.0f       /* 目标温度，陀螺零偏在约 40 °C 下标定 */
#define IMU_HEATER_SHUTDOWN_C               45.0f       /* 超过就停止加热（硬保护） */

#define IMU_HEATER_PREHEAT_DUTY             0.9f       /* 预热占空比（0..1） */
#define IMU_HEATER_PREHEAT_MARGIN_C         5.0f        /* 温度达到 目标 - 余量 开始计时 */
#define IMU_HEATER_PREHEAT_HOLD_CYCLES      20u         /* 累计满这么多个周期（10 Hz 下 2 s）后转入恒温 */

#define IMU_HEATER_KP                       0.1f        /* 单位：占空比 / °C */
#define IMU_HEATER_KI                       0.05f      /* 每个调用周期累加 */
#define IMU_HEATER_KD                       0.0f
#define IMU_HEATER_INTEGRAL_LIMIT           0.1f
#define IMU_HEATER_INTEGRAL_PRESET          0.1f       /* 转入恒温时的积分初值，不能超过 INTEGRAL_LIMIT */
#define IMU_HEATER_MAX_DUTY                 0.5f      /* 恒温段最大占空比 */
#define IMU_HEATER_DEAD_BAND_C              0.0f
#endif