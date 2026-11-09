/* IMU 恒温加热任务。
 * 10 Hz 周期任务：直接读取 BMI088 最近一次有效温度（不依赖姿态解算），交给 heater 做预热 + PID 恒温。
 * 保护：温度数据超过 HEATER_STALE_LIMIT_CYCLES 个周期没有更新（温度读取停止或 IMU 掉线）就关加热，防止在没有反馈时一直加热。
 * shell：heater 看阶段、目标/当前温度和占空比。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/log/log.h"
#include "cdd/imu/bmi088/bmi088.h"
#include "cdd/imu/heater/heater.h"
#include "cdd/imu/ins/ins.h"

#define HEATER_STALE_LIMIT_CYCLES   5u

static uint32_t last_temperature_sequence;
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
