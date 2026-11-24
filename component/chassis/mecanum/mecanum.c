#include "component/chassis/mecanum/mecanum.h"
#include "component/common/err.h"

int32_t mecanum_solve(const chassis_geometry_t *geometry, const chassis_speed_t *speed,
                      float wheel_rad_s[CHASSIS_WHEEL_COUNT])
{
    if (geometry == NULL || speed == NULL || wheel_rad_s == NULL ||
        geometry->half_length_m < 0.0f || geometry->half_width_m < 0.0f || !(geometry->wheel_radius_m > 0.0f))
    {
        return ERR_INVAL;
    }
    float spin = speed->spin_rad_s * (geometry->half_length_m + geometry->half_width_m);
    float scale = 1.0f / geometry->wheel_radius_m;
    float forward = speed->forward_m_s;
    float right = speed->right_m_s;

    wheel_rad_s[CHASSIS_WHEEL_FRONT_LEFT]  = (forward + right - spin) * scale;
    wheel_rad_s[CHASSIS_WHEEL_FRONT_RIGHT] = (forward - right + spin) * scale;
    wheel_rad_s[CHASSIS_WHEEL_REAR_RIGHT]  = (forward + right + spin) * scale;
    wheel_rad_s[CHASSIS_WHEEL_REAR_LEFT]   = (forward - right - spin) * scale;
    return ERR_OK;
}

int32_t mecanum_inverse(const chassis_geometry_t *geometry, const float wheel_rad_s[CHASSIS_WHEEL_COUNT],
                        chassis_speed_t *speed)
{
    if (geometry == NULL || wheel_rad_s == NULL || speed == NULL ||
        geometry->half_length_m < 0.0f || geometry->half_width_m < 0.0f ||
        !(geometry->half_length_m + geometry->half_width_m > 0.0f) || !(geometry->wheel_radius_m > 0.0f))
    {
        return ERR_INVAL;
    }

    float radius = geometry->wheel_radius_m;
    float front_left = wheel_rad_s[CHASSIS_WHEEL_FRONT_LEFT] * radius;
    float front_right = wheel_rad_s[CHASSIS_WHEEL_FRONT_RIGHT] * radius;
    float rear_right = wheel_rad_s[CHASSIS_WHEEL_REAR_RIGHT] * radius;
    float rear_left = wheel_rad_s[CHASSIS_WHEEL_REAR_LEFT] * radius;

    speed->forward_m_s = (front_left + front_right + rear_right + rear_left) * 0.25f;
    speed->right_m_s = (front_left - front_right + rear_right - rear_left) * 0.25f;
    speed->spin_rad_s = (-front_left + front_right + rear_right - rear_left) * 0.25f /
                        (geometry->half_length_m + geometry->half_width_m);
    return ERR_OK;
}
