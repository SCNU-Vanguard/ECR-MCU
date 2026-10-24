#ifndef CHASSIS_CONFIG_H__
#define CHASSIS_CONFIG_H__
#include "component/chassis/omni/omni.h"

/* 正方形四角布轮，边长 600 mm，轮半径 100 mm。
 * 解算用的半径是几何中心到轮心：(0.6 / 2) * sqrt(2)。
 * 轮序见 component/chassis/chassis.h。解算结果是轮端 rad/s。
 * m3508 速度反馈和速度目标都在转子侧，轮端角速度要乘这个减速比。 */
#define M3508_GEAR_RATIO         19.0f
#define OMNI_CHASSIS_RADIUS_M    0.42426407f
#define OMNI_WHEEL_RADIUS_M      0.10f

static const omni_geometry_t omni =
{
    .chassis_radius_m = OMNI_CHASSIS_RADIUS_M,
    .wheel_radius_m   = OMNI_WHEEL_RADIUS_M,
};
#endif
