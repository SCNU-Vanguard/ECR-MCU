#ifndef COMPONENT_CHASSIS_SWERVE_H__
#define COMPONENT_CHASSIS_SWERVE_H__
#include "component/chassis/chassis.h"

/* 四轮舵轮。每个轮心在 (±半轴距, ±半轮距)。
 * speed_rad_s 是轮端角速度，不含减速比。steer_rad 为 0 时轮子朝前，范围 (-pi, pi]。
 * 某一轮的平移速度和旋转切向速度都是 0 时，不改该轮传入的 steer_rad，只把速度写成 0，避免停车时舵角被打回 0。 */
typedef struct
{
    float half_length_m;
    float half_width_m;
    float wheel_radius_m;
} swerve_geometry_t;

typedef struct
{
    float speed_rad_s;
    float steer_rad;
} swerve_wheel_t;

int32_t swerve_solve(const swerve_geometry_t *geometry, const chassis_speed_t *speed,
                     swerve_wheel_t wheel[CHASSIS_WHEEL_COUNT]);
/* 相对当前舵角走短边。转角差超过 90° 时舵向加 π、轮速取反 */
void swerve_shortest(float current_steer_rad, swerve_wheel_t *wheel);
#endif
