#ifndef COMPONENT_CHASSIS_OMNI_H__
#define COMPONENT_CHASSIS_OMNI_H__
#include "component/chassis/chassis.h"

/* 四轮全向，辊子相对轮轴 45°，四轮到几何中心等距。
 * 平移分量是速度在辊子驱动方向上的投影，旋转分量是自旋角速度乘中心到轮心的距离。
 * 输出轮端角速度 rad/s，不含电机减速比。 */
typedef struct
{
    float chassis_radius_m;    /* 几何中心到轮心，不能为负 */
    float wheel_radius_m;
} omni_geometry_t;

int32_t omni_solve(const omni_geometry_t *geometry, const chassis_speed_t *speed,
                   float wheel_rad_s[CHASSIS_WHEEL_COUNT]);
#endif
