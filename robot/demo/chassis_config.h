#ifndef CHASSIS_CONFIG_H__
#define CHASSIS_CONFIG_H__
#include "component/chassis/chassis.h"

/* 构型。对应的 CONFIG_COMPONENT_CHASSIS_* 要在 robot.config 里打开，否则编译失败。
 * 全向轮，辊子 45°。轮子在矩形四角，轮序见 component/chassis/chassis.h。
 * 几何用半轴距、半轮距和轮半径。解算和减速比在 chassis 模块里，应用层只传电机。
 *
 * CHASSIS_DRIVE_RATIO：驱动电机的速度反馈、速度目标相对轮端角速度的倍数。
 * 一台车的驱动电机是同一个型号，所以只有一个值：
 *   m3508：速度在转子侧，减速比 19，填 19。
 *   s3519：速度已经是输出轴侧，驱动层内部处理了减速比，填 1。
 *   直驱电机：填 1。
 * 舵轮另有 CHASSIS_STEER_RATIO：转向电机角度相对轮端舵角的倍数，不写时按 1。 */
#define CHASSIS_TYPE             CHASSIS_TYPE_OMNI

#define CHASSIS_HALF_LENGTH_M    0.30f    /* 几何中心到前/后轮轴 */
#define CHASSIS_HALF_WIDTH_M     0.30f    /* 几何中心到左/右轮 */
#define CHASSIS_WHEEL_RADIUS_M   0.10f
#define CHASSIS_DRIVE_RATIO      19.0f

/* 只限制驱动电机。发射、云台、转向和关节电机由应用层决定不传进来。
 * 功率模型按驱动电机型号标定，一台车的驱动电机是同一型号，共用这一组系数。换电机型号要重新标定。
 * 系数来自 2026 步兵 M3508 模型。原式用转子 rpm，这里改成 rad/s：
 *   k_output_speed = 1.996889994e-6 * 9.5493
 *   k_speed2       = 2.0326e-7 * 9.5493^2
 * 能量高于 38（约 15%）时允许比裁判上限多 25 W，否则少 10 W。 */
#define CHASSIS_POWER_K_OUTPUT_SPEED   1.9069e-5f
#define CHASSIS_POWER_K_SPEED2         1.8535e-5f
#define CHASSIS_POWER_K_OUTPUT2        1.453e-7f
#define CHASSIS_POWER_IDLE_W           0.75f

#define CHASSIS_POWER_EXTRA_W          25.0f
#define CHASSIS_POWER_MARGIN_W         10.0f
#define CHASSIS_POWER_ENERGY_HIGH_RAW  38u
#endif
