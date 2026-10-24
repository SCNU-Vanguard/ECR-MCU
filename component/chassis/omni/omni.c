#include "component/chassis/omni/omni.h"
#include "component/common/err.h"

#define OMNI_ROLLER_COS  0.70710678118f

int32_t omni_solve(const omni_geometry_t *geometry, const chassis_speed_t *speed,
                   float wheel_rad_s[CHASSIS_WHEEL_COUNT])
{
    if (geometry == NULL || speed == NULL || wheel_rad_s == NULL ||
        geometry->chassis_radius_m < 0.0f || !(geometry->wheel_radius_m > 0.0f))
    {
        return ERR_INVAL;
    }
    float forward = speed->forward_m_s * OMNI_ROLLER_COS;
    float right = speed->right_m_s * OMNI_ROLLER_COS;
    float spin = speed->spin_rad_s * geometry->chassis_radius_m;
    float scale = 1.0f / geometry->wheel_radius_m;

    wheel_rad_s[CHASSIS_WHEEL_FRONT_LEFT]  = ( forward + right + spin) * scale;
    wheel_rad_s[CHASSIS_WHEEL_FRONT_RIGHT] = (-forward + right + spin) * scale;
    wheel_rad_s[CHASSIS_WHEEL_REAR_RIGHT]  = (-forward - right + spin) * scale;
    wheel_rad_s[CHASSIS_WHEEL_REAR_LEFT]   = ( forward - right + spin) * scale;
    return ERR_OK;
}

int32_t omni_inverse(const omni_geometry_t *geometry, const float wheel_rad_s[CHASSIS_WHEEL_COUNT],
                     chassis_speed_t *speed)
{
    if (geometry == NULL || wheel_rad_s == NULL || speed == NULL ||
        !(geometry->chassis_radius_m > 0.0f) || !(geometry->wheel_radius_m > 0.0f))
    {
        return ERR_INVAL;
    }

    float front_left = wheel_rad_s[CHASSIS_WHEEL_FRONT_LEFT];
    float front_right = wheel_rad_s[CHASSIS_WHEEL_FRONT_RIGHT];
    float rear_right = wheel_rad_s[CHASSIS_WHEEL_REAR_RIGHT];
    float rear_left = wheel_rad_s[CHASSIS_WHEEL_REAR_LEFT];
    float wheel_radius = geometry->wheel_radius_m;
    float forward_term = (front_left - front_right - rear_right + rear_left) * wheel_radius * 0.25f;
    float right_term = (front_left + front_right - rear_right - rear_left) * wheel_radius * 0.25f;
    float spin_term = (front_left + front_right + rear_right + rear_left) * wheel_radius * 0.25f;

    speed->forward_m_s = forward_term / OMNI_ROLLER_COS;
    speed->right_m_s = right_term / OMNI_ROLLER_COS;
    speed->spin_rad_s = spin_term / geometry->chassis_radius_m;
    return ERR_OK;
}
