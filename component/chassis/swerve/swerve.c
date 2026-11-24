#include <math.h>
#include "component/chassis/swerve/swerve.h"
#include "component/common/err.h"

#define SWERVE_PI       3.14159265f
#define SWERVE_STOP_M_S 1.0e-4f

static float wrap_pi(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

static bool geometry_ok(const chassis_geometry_t *geometry)
{
    return geometry != NULL &&
           geometry->half_length_m >= 0.0f &&
           geometry->half_width_m >= 0.0f &&
           (geometry->wheel_radius_m > 0.0f);
}

/* 轮心坐标：x 向前，y 向右。顺序和 CHASSIS_WHEEL_* 一致。 */
static void wheel_position(const chassis_geometry_t *geometry,
                           float wheel_x[CHASSIS_WHEEL_COUNT],
                           float wheel_y[CHASSIS_WHEEL_COUNT])
{
    wheel_x[CHASSIS_WHEEL_FRONT_RIGHT] = geometry->half_length_m;
    wheel_x[CHASSIS_WHEEL_FRONT_LEFT]  = geometry->half_length_m;
    wheel_x[CHASSIS_WHEEL_REAR_LEFT]   = -geometry->half_length_m;
    wheel_x[CHASSIS_WHEEL_REAR_RIGHT]  = -geometry->half_length_m;

    wheel_y[CHASSIS_WHEEL_FRONT_RIGHT] = geometry->half_width_m;
    wheel_y[CHASSIS_WHEEL_FRONT_LEFT]  = -geometry->half_width_m;
    wheel_y[CHASSIS_WHEEL_REAR_LEFT]   = -geometry->half_width_m;
    wheel_y[CHASSIS_WHEEL_REAR_RIGHT]  = geometry->half_width_m;
}

int32_t swerve_solve(const chassis_geometry_t *geometry, const chassis_speed_t *speed,
                     swerve_wheel_t wheel[CHASSIS_WHEEL_COUNT])
{
    float wheel_x[CHASSIS_WHEEL_COUNT];
    float wheel_y[CHASSIS_WHEEL_COUNT];

    if (!geometry_ok(geometry) || speed == NULL || wheel == NULL)
    {
        return ERR_INVAL;
    }
    wheel_position(geometry, wheel_x, wheel_y);
    for (uint32_t wheel_index = 0; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
    {
        /* 轮心速度 = 底盘中心速度 + 自旋 × 轮心位置 */
        float forward = speed->forward_m_s + speed->spin_rad_s * wheel_y[wheel_index];
        float right = speed->right_m_s - speed->spin_rad_s * wheel_x[wheel_index];
        float wheel_m_s = sqrtf(forward * forward + right * right);

        wheel[wheel_index].speed_rad_s = wheel_m_s / geometry->wheel_radius_m;
        if (wheel_m_s > SWERVE_STOP_M_S)
        {
            /* 舵角和自旋、yaw 一致，俯视逆时针为正：轮子朝右时是负角 */
            wheel[wheel_index].steer_rad = atan2f(-right, forward);
        }
    }
    return ERR_OK;
}

int32_t swerve_inverse(const chassis_geometry_t *geometry, const swerve_wheel_t wheel[CHASSIS_WHEEL_COUNT],
                       chassis_speed_t *speed)
{
    float wheel_x[CHASSIS_WHEEL_COUNT];
    float wheel_y[CHASSIS_WHEEL_COUNT];
    float sum_forward = 0.0f;
    float sum_right = 0.0f;
    float spin_num = 0.0f;
    float spin_den = 0.0f;

    if (!geometry_ok(geometry) || wheel == NULL || speed == NULL)
    {
        return ERR_INVAL;
    }
    wheel_position(geometry, wheel_x, wheel_y);
    for (uint32_t wheel_index = 0; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
    {
        /* steer = atan2(-right, forward)，轮缘速度沿这个方向 */
        float wheel_m_s = wheel[wheel_index].speed_rad_s * geometry->wheel_radius_m;
        float forward = wheel_m_s * cosf(wheel[wheel_index].steer_rad);
        float right = -wheel_m_s * sinf(wheel[wheel_index].steer_rad);

        sum_forward += forward;
        sum_right += right;
        spin_num += forward * wheel_y[wheel_index] - right * wheel_x[wheel_index];
        spin_den += wheel_x[wheel_index] * wheel_x[wheel_index] + wheel_y[wheel_index] * wheel_y[wheel_index];
    }
    speed->forward_m_s = sum_forward / (float)CHASSIS_WHEEL_COUNT;
    speed->right_m_s = sum_right / (float)CHASSIS_WHEEL_COUNT;
    speed->spin_rad_s = (spin_den > 0.0f) ? (spin_num / spin_den) : 0.0f;
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
