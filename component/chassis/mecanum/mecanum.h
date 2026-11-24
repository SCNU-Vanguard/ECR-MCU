#ifndef COMPONENT_CHASSIS_MECANUM_H__
#define COMPONENT_CHASSIS_MECANUM_H__
#include "component/chassis/chassis.h"

/* 45° 辊子的 X 型麦克纳姆。旋转项是半轴距 + 半轮距。
 * 输出轮端角速度 rad/s，不含电机减速比。
 * mecanum_inverse 由四个轮端角速度求底盘中心速度。几何用 chassis_geometry_t。 */

/* 半轴距、半轮距为负，或轮半径不是正数：返回 ERR_INVAL，不写输出。
 * 逆解算还要求半轴距 + 半轮距大于 0。 */
int32_t mecanum_solve(const chassis_geometry_t *geometry, const chassis_speed_t *speed,
                      float wheel_rad_s[CHASSIS_WHEEL_COUNT]);
int32_t mecanum_inverse(const chassis_geometry_t *geometry, const float wheel_rad_s[CHASSIS_WHEEL_COUNT],
                        chassis_speed_t *speed);
#endif
