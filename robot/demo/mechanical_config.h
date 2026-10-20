#ifndef MECHANICAL_CONFIG_H__
#define MECHANICAL_CONFIG_H__
#include "component/chassis/mecanum/mecanum.h"

/* demo 底盘是麦克纳姆，辊子 45°。长度还没实测。
 * 轮序见 component/chassis/chassis.h。解算结果是轮端 rad/s。
 * m3508 速度反馈和速度目标都在转子侧，轮端角速度要乘这个减速比。 */
#define M3508_GEAR_RATIO  19.0f

static const mecanum_geometry_t mecanum =
{
    .half_length_m  = 0.60f,
    .half_width_m   = 0.40f,
    .wheel_radius_m = 0.10f,
};
#endif
