#include "cdd/imu/heater/heater.h"
#include "ins_config.h"
#include "bsp/pwm/bsp_pwm.h"
#include "component/common/err.h"

#define PPM_FULL_SCALE      1000000.0f

static pid_controller_t heater_pid;
static float last_temperature_c;
static uint32_t duty_ppm;
static bool preheating;
static bool startup_temperature_pending;
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
    preheating = false;
    startup_temperature_pending = true;
    cutoff_active = false;
    last_temperature_c = 0.0f;
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
    if (temperature_c >= IMU_HEATER_SHUTDOWN_C)
    {
        cutoff_active = true;
        /* 高温启动或运行中过温后锁定退出预热，降温时不得重新施加预热功率。 */
        preheating = false;
        startup_temperature_pending = false;
        pid_clear(&heater_pid);
        apply_duty(0.0f);
        return;
    }
    cutoff_active = false;
    if (startup_temperature_pending)
    {
        startup_temperature_pending = false;

        // 预加热到40度，之后切换成普通加热，这个时候一般会飙到50多度，之后热量冗余，会使得pid温控跳变没有这么大
        preheating = temperature_c < IMU_HEATER_TARGET_C - IMU_HEATER_PREHEAT_MARGIN_C;
    }
    if (preheating)
    {
        if (temperature_c >= IMU_HEATER_TARGET_C - IMU_HEATER_PREHEAT_MARGIN_C)
        {
            preheating = false;
            heater_pid.integral = IMU_HEATER_INTEGRAL_PRESET;
        }
        else
        {
            apply_duty(IMU_HEATER_PREHEAT_DUTY);
            return;
        }
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
