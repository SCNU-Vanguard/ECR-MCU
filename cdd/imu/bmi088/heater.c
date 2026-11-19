#include "cdd/imu/bmi088/heater.h"
#include "cdd/imu/bmi088/bmi088.h"
#include "ins_config.h"
#include "osal/period.h"
#include "bsp/pwm/bsp_pwm.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/log/log.h"
#include "component/pid/pid.h"

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
        .kp = HEATER_KP,
        .ki = HEATER_KI,
        .kd = HEATER_KD,
        .output_limit = HEATER_MAX_DUTY,
        .integral_limit = HEATER_INTEGRAL_LIMIT,
        .dead_band = HEATER_DEAD_BAND_C,
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
    if (temperature_c >= HEATER_SHUTDOWN_C)
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
        preheating = temperature_c < HEATER_TARGET_C - HEATER_PREHEAT_MARGIN_C;
    }
    if (preheating)
    {
        if (temperature_c >= HEATER_TARGET_C - HEATER_PREHEAT_MARGIN_C)
        {
            preheating = false;
            heater_pid.integral = HEATER_INTEGRAL_PRESET;
        }
        else
        {
            apply_duty(HEATER_PREHEAT_DUTY);
            return;
        }
    }
    (void)pid_position_compute(&heater_pid, temperature_c, HEATER_TARGET_C);
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
    return HEATER_TARGET_C;
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

/* 10 Hz：读 BMI088 最近一次有效温度，做预热 + PID 恒温。
 * 温度超过 HEATER_STALE_LIMIT_CYCLES 个周期没有更新就关加热，避免没有反馈时一直加热。 */
#define HEATER_STALE_LIMIT_CYCLES   5u

static uint32_t last_temperature_sequence;
static uint32_t stale_cycles;
static bool heater_ready;

static int32_t heater_module_init(void)
{
    int32_t ret = heater_init();

    heater_ready = (ret == ERR_OK);
    if (!heater_ready)
    {
        LOGE("init failed: %d", (int)ret);
    }
    return ret;
}
INIT_CDD_STAGE2(heater_module_init);

static void heater_run(void *argument)
{
    float temperature_c;
    uint32_t temperature_sequence;

    (void)argument;
    if (!heater_ready)
    {
        return;
    }
    if (bmi088_get_temperature(&temperature_c, &temperature_sequence) &&
        temperature_sequence != last_temperature_sequence)
    {
        last_temperature_sequence = temperature_sequence;
        stale_cycles = 0u;
        heater_step(temperature_c);
        return;
    }
    if (stale_cycles < HEATER_STALE_LIMIT_CYCLES)
    {
        stale_cycles++;
    }
    if (stale_cycles >= HEATER_STALE_LIMIT_CYCLES)
    {
        heater_off();
    }
}
PERIOD_TASK_DEFINE(heater_task, OS_FREQ_10HZ, OS_PRIORITY_LOW, OS_STACK_SMALL, heater_run, NULL);

#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"

static bool heater_input_stale(void)
{
    return stale_cycles >= HEATER_STALE_LIMIT_CYCLES;
}

static void print_milli(const char *label, float value, const char *unit)
{
    long milli = (long)(value * 1000.0f + (value >= 0.0f ? 0.5f : -0.5f));
    const char *sign = (milli < 0) ? "-" : "";

    if (milli < 0)
    {
        milli = -milli;
    }
    shell_out("  %s = %s%ld.%03ld %s\r\n", label, sign, milli / 1000, milli % 1000, unit);
}

static int shell_heater(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    uint32_t duty_ppm_now = heater_duty_ppm();

    shell_out("heater: ready=%d phase=%s input_stale=%d cutoff=%d\r\n", (int)heater_ready,
              heater_preheating() ? "preheat" : "hold", (int)heater_input_stale(), (int)heater_cutoff_active());
    print_milli("target", heater_target_c(), "C");
    print_milli("temperature", heater_last_temperature_c(), "C");
    shell_out("  duty = %lu.%02lu %%\r\n", (unsigned long)(duty_ppm_now / 10000u),
              (unsigned long)((duty_ppm_now % 10000u) / 100u));
    return 0;
}
SHELL_CMD(heater, shell_heater, show_heater_status)
#endif
