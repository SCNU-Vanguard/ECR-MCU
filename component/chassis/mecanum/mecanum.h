#ifndef COMPONENT_CHASSIS_MECANUM_H__
#define COMPONENT_CHASSIS_MECANUM_H__
#include "component/chassis/chassis.h"

/* 45° 辊子的 X 型麦克纳姆。旋转项是半轴距 + 半轮距。
 * 输出轮端角速度 rad/s，不含电机减速比。 */
typedef struct
{
    float half_length_m;    /* 几何中心到前/后轮轴 */
    float half_width_m;     /* 几何中心到左/右轮 */
    float wheel_radius_m;
} mecanum_geometry_t;

/* 半轴距、半轮距为负，或轮半径不是正数：返回 ERR_INVAL，不写输出 */
int32_t mecanum_solve(const mecanum_geometry_t *geometry, const chassis_speed_t *speed,
                      float wheel_rad_s[CHASSIS_WHEEL_COUNT]);
#endif
