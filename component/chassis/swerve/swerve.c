#include <math.h>
#include "component/chassis/swerve/swerve.h"
#include "component/common/err.h"

#define SWERVE_PI       3.14159265f
#define SWERVE_STOP_M_S 1.0e-4f

static float wrap_pi(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

int32_t swerve_solve(const swerve_geometry_t *geometry, const chassis_speed_t *speed,
                     swerve_wheel_t wheel[CHASSIS_WHEEL_COUNT])
{
    if (geometry == NULL || speed == NULL || wheel == NULL ||
        geometry->half_length_m < 0.0f || geometry->half_width_m < 0.0f || !(geometry->wheel_radius_m > 0.0f))
    {
        return ERR_INVAL;
    }
    /* 轮心坐标：x 向前，y 向右。轮心速度 = 底盘中心速度 + 自旋 × 轮心位置 */
    float wheel_x[CHASSIS_WHEEL_COUNT] =
    {
        geometry->half_length_m,
        geometry->half_length_m,
        -geometry->half_length_m,
        -geometry->half_length_m,
    };
    float wheel_y[CHASSIS_WHEEL_COUNT] =
    {
        geometry->half_width_m,
        -geometry->half_width_m,
        -geometry->half_width_m,
        geometry->half_width_m,
    };

    for (uint32_t wheel_index = 0; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
    {
        float forward = speed->forward_m_s + speed->spin_rad_s * wheel_y[wheel_index];
        float right = speed->right_m_s - speed->spin_rad_s * wheel_x[wheel_index];
        float wheel_m_s = sqrtf(forward * forward + right * right);

        wheel[wheel_index].speed_rad_s = wheel_m_s / geometry->wheel_radius_m;
        if (wheel_m_s > SWERVE_STOP_M_S)
        {
            wheel[wheel_index].steer_rad = atan2f(right, forward);
        }
    }
    return ERR_OK;
}

void swerve_shortest(float current_steer_rad, swerve_wheel_t *wheel)
{
    if (wheel == NULL)
    {
        return;
    }
    float error = wrap_pi(wheel->steer_rad - current_steer_rad);

    if (error > (SWERVE_PI * 0.5f) || error < -(SWERVE_PI * 0.5f))
    {
        wheel->steer_rad = wrap_pi(wheel->steer_rad + SWERVE_PI);
        wheel->speed_rad_s = -wheel->speed_rad_s;
    }
}
