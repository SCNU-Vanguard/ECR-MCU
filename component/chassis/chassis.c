#include "component/chassis/chassis.h"
#include "component/common/err.h"
#include "chassis_config.h"

#ifndef CHASSIS_TYPE
#error "chassis_config.h must define CHASSIS_TYPE"
#endif
#ifndef CHASSIS_HALF_LENGTH_M
#error "chassis_config.h must define CHASSIS_HALF_LENGTH_M"
#endif
#ifndef CHASSIS_HALF_WIDTH_M
#error "chassis_config.h must define CHASSIS_HALF_WIDTH_M"
#endif
#ifndef CHASSIS_WHEEL_RADIUS_M
#error "chassis_config.h must define CHASSIS_WHEEL_RADIUS_M"
#endif
#ifndef CHASSIS_DRIVE_RATIO
#error "chassis_config.h must define CHASSIS_DRIVE_RATIO"
#endif

#if CHASSIS_TYPE == CHASSIS_TYPE_OMNI
#ifndef CONFIG_COMPONENT_CHASSIS_OMNI
#error "CHASSIS_TYPE is omni but CONFIG_COMPONENT_CHASSIS_OMNI is off"
#endif
#include "component/chassis/omni/omni.h"
#elif CHASSIS_TYPE == CHASSIS_TYPE_MECANUM
#ifndef CONFIG_COMPONENT_CHASSIS_MECANUM
#error "CHASSIS_TYPE is mecanum but CONFIG_COMPONENT_CHASSIS_MECANUM is off"
#endif
#include "component/chassis/mecanum/mecanum.h"
#elif CHASSIS_TYPE == CHASSIS_TYPE_SWERVE
#ifndef CONFIG_COMPONENT_CHASSIS_SWERVE
#error "CHASSIS_TYPE is swerve but CONFIG_COMPONENT_CHASSIS_SWERVE is off"
#endif
#include "component/chassis/swerve/swerve.h"
#ifndef CHASSIS_STEER_RATIO
#define CHASSIS_STEER_RATIO 1.0f
#endif
#else
#error "CHASSIS_TYPE must be CHASSIS_TYPE_OMNI, CHASSIS_TYPE_MECANUM, or CHASSIS_TYPE_SWERVE"
#endif

#ifdef CONFIG_COMPONENT_POWER
#include "component/power/power.h"
#ifndef CHASSIS_POWER_K_OUTPUT_SPEED
#error "chassis_config.h must define CHASSIS_POWER_K_OUTPUT_SPEED"
#endif
#ifndef CHASSIS_POWER_K_SPEED2
#error "chassis_config.h must define CHASSIS_POWER_K_SPEED2"
#endif
#ifndef CHASSIS_POWER_K_OUTPUT2
#error "chassis_config.h must define CHASSIS_POWER_K_OUTPUT2"
#endif
#ifndef CHASSIS_POWER_IDLE_W
#error "chassis_config.h must define CHASSIS_POWER_IDLE_W"
#endif
#ifndef CHASSIS_POWER_EXTRA_W
#error "chassis_config.h must define CHASSIS_POWER_EXTRA_W"
#endif
#ifndef CHASSIS_POWER_MARGIN_W
#error "chassis_config.h must define CHASSIS_POWER_MARGIN_W"
#endif
#ifndef CHASSIS_POWER_ENERGY_HIGH_RAW
#error "chassis_config.h must define CHASSIS_POWER_ENERGY_HIGH_RAW"
#endif
#endif

static const chassis_geometry_t geometry =
{
    .half_length_m = CHASSIS_HALF_LENGTH_M,
    .half_width_m = CHASSIS_HALF_WIDTH_M,
    .wheel_radius_m = CHASSIS_WHEEL_RADIUS_M,
};

#ifdef CONFIG_COMPONENT_POWER
static const power_model_t power_model =
{
    .k_output_speed = CHASSIS_POWER_K_OUTPUT_SPEED,
    .k_speed2 = CHASSIS_POWER_K_SPEED2,
    .k_output2 = CHASSIS_POWER_K_OUTPUT2,
    .idle_w = CHASSIS_POWER_IDLE_W,
};
#endif

static int32_t require_drive(motor_t *const drive[CHASSIS_WHEEL_COUNT])
{
    if (drive == NULL || !(CHASSIS_DRIVE_RATIO > 0.0f))
    {
        return ERR_INVAL;
    }
    for (uint32_t index = 0; index < CHASSIS_WHEEL_COUNT; index++)
    {
        if (drive[index] == NULL)
        {
            return ERR_INVAL;
        }
    }
    return ERR_OK;
}

#if CHASSIS_TYPE != CHASSIS_TYPE_SWERVE
static int32_t read_drive(motor_t *const drive[CHASSIS_WHEEL_COUNT], float wheel_rad_s[CHASSIS_WHEEL_COUNT])
{
    int32_t ret = require_drive(drive);

    if (ret != ERR_OK)
    {
        return ret;
    }
    for (uint32_t index = 0; index < CHASSIS_WHEEL_COUNT; index++)
    {
        if (!motor_online(drive[index]))
        {
            return ERR_FAIL;
        }
        wheel_rad_s[index] = drive[index]->speed_rad_s / CHASSIS_DRIVE_RATIO;
    }
    return ERR_OK;
}

static void write_drive(motor_t *const drive[CHASSIS_WHEEL_COUNT], const float wheel_rad_s[CHASSIS_WHEEL_COUNT])
{
    for (uint32_t index = 0; index < CHASSIS_WHEEL_COUNT; index++)
    {
        (void)motor_set_target(drive[index], wheel_rad_s[index] * CHASSIS_DRIVE_RATIO);
    }
}
#endif

#if CHASSIS_TYPE == CHASSIS_TYPE_SWERVE
static int32_t require_steer(motor_t *const steer[CHASSIS_WHEEL_COUNT])
{
    if (steer == NULL)
    {
        return ERR_INVAL;
    }
    for (uint32_t index = 0; index < CHASSIS_WHEEL_COUNT; index++)
    {
        if (steer[index] == NULL)
        {
            return ERR_INVAL;
        }
    }
    return ERR_OK;
}

static int32_t solve_swerve(motor_t *const drive[CHASSIS_WHEEL_COUNT],
                            motor_t *const steer[CHASSIS_WHEEL_COUNT],
                            const chassis_speed_t *speed)
{
    swerve_wheel_t wheel[CHASSIS_WHEEL_COUNT];
    int32_t ret = require_steer(steer);

    if (ret != ERR_OK || !(CHASSIS_STEER_RATIO > 0.0f))
    {
        return ERR_INVAL;
    }
    for (uint32_t index = 0; index < CHASSIS_WHEEL_COUNT; index++)
    {
        wheel[index].speed_rad_s = 0.0f;
        wheel[index].steer_rad = steer[index]->single_angle_rad / CHASSIS_STEER_RATIO;
    }
    ret = swerve_solve(&geometry, speed, wheel);
    if (ret != ERR_OK)
    {
        return ret;
    }
    for (uint32_t index = 0; index < CHASSIS_WHEEL_COUNT; index++)
    {
        float current_steer_rad = steer[index]->single_angle_rad / CHASSIS_STEER_RATIO;

        swerve_shortest(current_steer_rad, &wheel[index]);
        (void)motor_set_target(drive[index], wheel[index].speed_rad_s * CHASSIS_DRIVE_RATIO);
        (void)motor_set_target(steer[index], wheel[index].steer_rad * CHASSIS_STEER_RATIO);
    }
    return ERR_OK;
}

static int32_t inverse_swerve(motor_t *const drive[CHASSIS_WHEEL_COUNT],
                              motor_t *const steer[CHASSIS_WHEEL_COUNT],
                              chassis_speed_t *speed)
{
    swerve_wheel_t wheel[CHASSIS_WHEEL_COUNT];
    int32_t ret = require_drive(drive);

    if (ret != ERR_OK || !(CHASSIS_STEER_RATIO > 0.0f))
    {
        return ERR_INVAL;
    }
    ret = require_steer(steer);
    if (ret != ERR_OK)
    {
        return ret;
    }
    for (uint32_t index = 0; index < CHASSIS_WHEEL_COUNT; index++)
    {
        if (!motor_online(drive[index]) || !motor_online(steer[index]))
        {
            return ERR_FAIL;
        }
        wheel[index].speed_rad_s = drive[index]->speed_rad_s / CHASSIS_DRIVE_RATIO;
        wheel[index].steer_rad = steer[index]->single_angle_rad / CHASSIS_STEER_RATIO;
    }
    return swerve_inverse(&geometry, wheel, speed);
}
#endif

#if CHASSIS_TYPE == CHASSIS_TYPE_SWERVE
int32_t chassis_solve(motor_t *const drive[CHASSIS_WHEEL_COUNT],
                      motor_t *const steer[CHASSIS_WHEEL_COUNT],
                      const chassis_speed_t *speed)
{
    int32_t ret = require_drive(drive);

    if (ret != ERR_OK || speed == NULL)
    {
        return ERR_INVAL;
    }
    return solve_swerve(drive, steer, speed);
}

int32_t chassis_inverse(motor_t *const drive[CHASSIS_WHEEL_COUNT],
                        motor_t *const steer[CHASSIS_WHEEL_COUNT],
                        chassis_speed_t *speed)
{
    if (speed == NULL)
    {
        return ERR_INVAL;
    }
    return inverse_swerve(drive, steer, speed);
}
#else
int32_t chassis_solve(motor_t *const drive[CHASSIS_WHEEL_COUNT], const chassis_speed_t *speed)
{
    float wheel_rad_s[CHASSIS_WHEEL_COUNT];
    int32_t ret = require_drive(drive);

    if (ret != ERR_OK || speed == NULL)
    {
        return ERR_INVAL;
    }
#if CHASSIS_TYPE == CHASSIS_TYPE_OMNI
    ret = omni_solve(&geometry, speed, wheel_rad_s);
#elif CHASSIS_TYPE == CHASSIS_TYPE_MECANUM
    ret = mecanum_solve(&geometry, speed, wheel_rad_s);
#endif
    if (ret != ERR_OK)
    {
        return ret;
    }
    write_drive(drive, wheel_rad_s);
    return ERR_OK;
}

int32_t chassis_inverse(motor_t *const drive[CHASSIS_WHEEL_COUNT], chassis_speed_t *speed)
{
    float wheel_rad_s[CHASSIS_WHEEL_COUNT];
    int32_t ret;

    if (speed == NULL)
    {
        return ERR_INVAL;
    }
    ret = read_drive(drive, wheel_rad_s);
    if (ret != ERR_OK)
    {
        return ret;
    }
#if CHASSIS_TYPE == CHASSIS_TYPE_OMNI
    return omni_inverse(&geometry, wheel_rad_s, speed);
#elif CHASSIS_TYPE == CHASSIS_TYPE_MECANUM
    return mecanum_inverse(&geometry, wheel_rad_s, speed);
#endif
}
#endif

float chassis_power_budget_w(const chassis_power_source_t *source)
{
#ifdef CONFIG_COMPONENT_POWER
    power_budget_t budget = {0};

    if (source == NULL)
    {
        return 0.0f;
    }
    budget.referee_valid = source->referee_valid;
    budget.referee_limit_w = source->referee_limit_w;
    budget.supercap_valid = source->supercap_valid;
    budget.energy_raw = source->energy_raw;
    budget.supercap_limit_w = source->supercap_limit_w;
    budget.extra_w = CHASSIS_POWER_EXTRA_W;
    budget.margin_w = CHASSIS_POWER_MARGIN_W;
    budget.energy_high_raw = CHASSIS_POWER_ENERGY_HIGH_RAW;
    return power_budget_w(&budget);
#else
    (void)source;
    return 0.0f;
#endif
}

float chassis_power_apply(motor_t *const motor[], uint8_t count, float max_power_w)
{
#ifdef CONFIG_COMPONENT_POWER
    power_channel_t channels[16];

    if (motor == NULL || count == 0u || count > 16u || max_power_w <= 0.0f)
    {
        return 0.0f;
    }
    for (uint8_t index = 0; index < count; index++)
    {
        channels[index].output = NULL;
        channels[index].speed_rad_s = 0.0f;
        if (motor[index] == NULL)
        {
            continue;
        }
        channels[index].output = &motor[index]->output;
        /* 模型按电机反馈的 rad/s 标定。m3508 是转子侧，这里不再除驱动减速比。 */
        channels[index].speed_rad_s = motor[index]->speed_rad_s;
    }
    return power_apply(channels, count, &power_model, max_power_w);
#else
    (void)motor;
    (void)count;
    (void)max_power_w;
    return 0.0f;
#endif
}
