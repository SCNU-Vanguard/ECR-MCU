#include "cdd/imu/heater/heater.h"
#include "cdd/imu/heater/heater_config.h"
#include "bsp/pwm/bsp_pwm.h"
#include "component/common/err.h"

#define PPM_FULL_SCALE      1000000.0f

static pid_controller_t heater_pid;
static float last_temperature_c;
static uint32_t duty_ppm;
static uint32_t preheat_hold_count;
static bool preheating;
static bool cutoff_active;
static bool initialized;

static void apply_duty(float ratio)
{
    if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }
    if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }
    duty_ppm = (uint32_t)(ratio * PPM_FULL_SCALE);
    (void)bsp_pwm_set_duty_ppm(PWM_HEATER, duty_ppm);
}

int32_t heater_init(void)
{
    const pid_config_t pid_config =
    {
        .kp = IMU_HEATER_KP,
        .ki = IMU_HEATER_KI,
        .kd = IMU_HEATER_KD,
        .output_limit = IMU_HEATER_MAX_DUTY,
        .integral_limit = IMU_HEATER_INTEGRAL_LIMIT,
        .dead_band = IMU_HEATER_DEAD_BAND_C,
    };

    pid_init(&heater_pid, &pid_config);
    preheating = true;
    preheat_hold_count = 0u;
    apply_duty(0.0f);
    int32_t ret = bsp_pwm_start(PWM_HEATER);

    initialized = (ret == ERR_OK);
    return ret;
}

void heater_step(float temperature_c)
{
    if (!initialized)
    {
        return;
    }
    last_temperature_c = temperature_c;
    if (temperature_c > IMU_HEATER_SHUTDOWN_C)
    {
        cutoff_active = true;
        apply_duty(0.0f);
        return;
    }
    cutoff_active = false;
    if (preheating)
    {
        apply_duty(IMU_HEATER_PREHEAT_DUTY);
        if (temperature_c > IMU_HEATER_TARGET_C - IMU_HEATER_PREHEAT_MARGIN_C)
        {
            preheat_hold_count++;
            if (preheat_hold_count >= IMU_HEATER_PREHEAT_HOLD_CYCLES)
            {
                preheating = false;
                heater_pid.integral = IMU_HEATER_INTEGRAL_PRESET;
            }
        }
        return;
    }
    (void)pid_position_compute(&heater_pid, temperature_c, IMU_HEATER_TARGET_C);
    if (heater_pid.output < 0.0f)                       /* 只能加热：输出和积分都不允许为负 */
    {
        heater_pid.output = 0.0f;
    }
    if (heater_pid.integral < 0.0f)
    {
        heater_pid.integral = 0.0f;
    }
    apply_duty(heater_pid.output);
}

void heater_off(void)
{
    if (initialized)
    {
        apply_duty(0.0f);
    }
}

float heater_target_c(void)
{
    return IMU_HEATER_TARGET_C;
}

float heater_last_temperature_c(void)
{
    return last_temperature_c;
}

uint32_t heater_duty_ppm(void)
{
    return duty_ppm;
}

bool heater_preheating(void)
{
    return preheating;
}

bool heater_cutoff_active(void)
{
    return cutoff_active;
}