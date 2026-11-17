#ifndef COMPONENT_CHASSIS_OMNI_H__
#define COMPONENT_CHASSIS_OMNI_H__
#include "component/chassis/chassis.h"

/* 四轮全向，轮子在矩形四角，辊子相对轮轴固定 45°。
 * 平移分量是速度在辊子驱动方向上的投影，旋转分量是自旋角速度乘（半轴距 + 半轮距）再投影到辊子方向。
 * 输出轮端角速度 rad/s，不含电机减速比。
 * omni_inverse 是上面这组式子的逆：由四个轮端角速度求底盘中心速度。
 * 几何参数和 mecanum_geometry_t、swerve_geometry_t 的字段相同。 */

typedef struct
{
    float half_length_m;    /* 几何中心到前/后轮轴 */
    float half_width_m;     /* 几何中心到左/右轮 */
    float wheel_radius_m;
} omni_geometry_t;

/* 半轴距、半轮距为负，或轮半径不是正数：返回 ERR_INVAL，不写输出。逆解算还要求半轴距 + 半轮距大于 0 */
int32_t omni_solve(const omni_geometry_t *geometry, const chassis_speed_t *speed,
                   float wheel_rad_s[CHASSIS_WHEEL_COUNT]);
int32_t omni_inverse(const omni_geometry_t *geometry, const float wheel_rad_s[CHASSIS_WHEEL_COUNT],
                     chassis_speed_t *speed);
#endif
