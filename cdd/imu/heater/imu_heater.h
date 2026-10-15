#ifndef CDD_IMU_HEATER_H__
#define CDD_IMU_HEATER_H__
#include <stdbool.h>
#include <stdint.h>
#include "component/pid/pid.h"

/* IMU 恒温加热：加热电阻接 PWM_HEATER（PB1 / TIM3_CH4），先大占空比预热，再用位置式 PID 把 IMU 温度控制到目标值。参数见 imu_heater_config.h。
 * 只能加热不能降温，所以恒温段输出限制在 [0, 最大占空比]。固定周期调用 imu_heater_step，PID 的 ki/kd 按这个周期整定。 */
int32_t imu_heater_init(void);                                   /* 按 imu_heater_config.h 的参数启动 PWM，占空比 0，从预热段开始 */
void    imu_heater_step(float temperature_c);                    /* 用最新温度算一次并更新占空比 */
void    imu_heater_off(void);                                    /* 占空比清 0（温度数据失效时调用），控制状态保留 */

float    imu_heater_target_c(void);
float    imu_heater_last_temperature_c(void);
uint32_t imu_heater_duty_ppm(void);                              /* 当前占空比，百万分比 */
bool     imu_heater_preheating(void);                            /* 还在预热段 */
bool     imu_heater_cutoff_active(void);                         /* 最近一次 step 是否因过温停止加热 */
#endif