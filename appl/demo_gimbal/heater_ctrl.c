/* 云台板：IMU 恒温。温度超过 0.5 s 不更新就关加热。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/log/log.h"
#include "cdd/imu/bmi088/bmi088.h"
#include "cdd/imu/heater/heater.h"
#include "cdd/imu/ins/ins.h"

#define HEATER_STALE_LIMIT_CYCLES   5u

static uint32_t last_update_count;
static uint32_t stale_cycles;
static bool heater_ready;

static int32_t heater_ctrl_init(void)
{
    int32_t ret = heater_init();

    heater_ready = (ret == ERR_OK);
    if (!heater_ready)
    {
        LOGE("init failed: %d", (int)ret);
    }
    return ret;
}
INIT_APP_STAGE2(heater_ctrl_init);

static void heater_ctrl_run(void *argument)
{
    ins_attitude_t attitude;
    bmi088_stats_t driver_stats;

    (void)argument;
    if (!heater_ready)
    {
        return;
    }
    ins_get_attitude(&attitude);
    bmi088_get_stats(&driver_stats);
    if (attitude.valid && driver_stats.temperature_count > 0u && attitude.update_count != last_update_count)
    {
        last_update_count = attitude.update_count;
        stale_cycles = 0u;
        heater_step(attitude.temperature_c);
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
PERIOD_TASK_DEFINE(heater_ctrl_task, OS_FREQ_10HZ, OS_PRIORITY_LOW, OS_STACK_SMALL, heater_ctrl_run, NULL);

static bool heater_input_stale(void)
{
    return stale_cycles >= HEATER_STALE_LIMIT_CYCLES;
}

#ifdef CONFIG_COMPONENT_SHELL
#include <stdio.h>
#include "component/shell/shell_cmd.h"

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
    uint32_t duty_ppm = heater_duty_ppm();

    shell_out("heater: ready=%d phase=%s input_stale=%d cutoff=%d\r\n", (int)heater_ready,
              heater_preheating() ? "preheat" : "hold", (int)heater_input_stale(), (int)heater_cutoff_active());
    print_milli("target", heater_target_c(), "C");
    print_milli("temperature", heater_last_temperature_c(), "C");
    shell_out("  duty = %lu.%02lu %%\r\n", (unsigned long)(duty_ppm / 10000u), (unsigned long)((duty_ppm % 10000u) / 100u));
    return 0;
}
SHELL_CMD(heater, shell_heater, show_heater_status)
#endif
