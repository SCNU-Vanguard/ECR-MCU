/* 底盘：CAN2 四个 m3508，麦克纳姆，另有一台 MF9025 和一个超级电容。左摇杆平移，拨轮只开关恒定自旋。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/remote/dt7/dt7.h"
#include "cdd/motor/motor_init.h"
#include "cdd/supercap/supercap.h"
#include "cdd/referee/referee.h"
#if defined(CONFIG_COMPONENT_POWER)
#include "component/power/power.h"
#endif
#include "chassis_config.h"
#include "supercap_config.h"
#include "power_config.h"

#define CHASSIS_STICK_MAX       660.0f
#define CHASSIS_STICK_IGNORE    20
#define CHASSIS_DIAL_IGNORE     100
#define CHASSIS_SPEED_MAX_M_S   1.5f
#define CHASSIS_SPIN_RAD_S      2.0f

static dji_motor_t *const chassis_wheel[CHASSIS_WHEEL_COUNT] =
{
    &chassis_m3508_0,
    &chassis_m3508_1,
    &chassis_m3508_2,
    &chassis_m3508_3,
};

#if defined(CONFIG_COMPONENT_POWER)
static const power_model_t power_model = POWER_MODEL_INIT;

static float chassis_power_limit_w(void)
{
    power_budget_t budget = {0};

#if defined(CONFIG_CDD_REFEREE)
    static referee_info_t referee;

    referee_get(&referee);
    if (referee.online && referee.performance.chassis_power_limit != 0u)
    {
        budget.referee_valid = true;
        budget.referee_limit_w = (float)referee.performance.chassis_power_limit;
    }
#endif
#if defined(CONFIG_CDD_SUPERCAP)
    supercap_status_t supercap;

    supercap_get(&supercap);
    if (supercap.online && supercap.error != SUPERCAP_ERROR_FATAL)
    {
        budget.supercap_valid = true;
        budget.energy_raw = supercap.energy_raw;
        budget.supercap_limit_w = (float)supercap.chassis_power_limit_w;
        budget.extra_w = POWER_EXTRA_W;
        budget.margin_w = POWER_MARGIN_W;
        budget.energy_high_raw = POWER_ENERGY_HIGH_RAW;
    }
#endif
    return power_budget_w(&budget);
}

static void chassis_power_apply(void)
{
    /* 要加功率限制的电机写在这个表里。MF9025 不在表内，output 不会被改。 */
    motor_t *motors[POWER_MOTOR_COUNT] =
    {
        &chassis_m3508_0.base,
        &chassis_m3508_1.base,
        &chassis_m3508_2.base,
        &chassis_m3508_3.base,
    };
    power_channel_t channels[POWER_MOTOR_COUNT];
    uint8_t index;

    for (index = 0; index < POWER_MOTOR_COUNT; index++)
    {
        channels[index].output = &motors[index]->output;
        channels[index].speed_rad_s = motors[index]->speed_rad_s;
    }
    (void)power_apply(channels, POWER_MOTOR_COUNT, &power_model, chassis_power_limit_w());
}
#endif

static float stick_to_speed(int16_t raw)
{
    if (raw > -CHASSIS_STICK_IGNORE && raw < CHASSIS_STICK_IGNORE)
    {
        return 0.0f;
    }
    return (float)raw / CHASSIS_STICK_MAX * CHASSIS_SPEED_MAX_M_S;
}

static int32_t chassis_ctrl_init(void)
{
    for (uint32_t wheel_index = 0; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
    {
        motor_set_active(&chassis_wheel[wheel_index]->base, true);
    }
    (void)lk_motor_run(&mf9025);
    motor_set_active(&mf9025.base, true);
    supercap_enable(true);
    supercap_set_referee(SUPERCAP_POWER_W, SUPERCAP_BUFFER_J);
    return ERR_OK;
}
INIT_APP_STAGE2(chassis_ctrl_init);

static void chassis_ctrl_run(void *argument)
{
    dt7_data_t remote;
    chassis_speed_t speed;
    float wheel_rad_s[CHASSIS_WHEEL_COUNT];

    (void)argument;
    dt7_get(&remote);
    speed.forward_m_s = stick_to_speed(remote.channel[3]);
    speed.right_m_s = stick_to_speed(remote.channel[2]);
    if (remote.wheel > CHASSIS_DIAL_IGNORE)
    {
        speed.spin_rad_s = CHASSIS_SPIN_RAD_S;
    }
    else if (remote.wheel < -CHASSIS_DIAL_IGNORE)
    {
        speed.spin_rad_s = -CHASSIS_SPIN_RAD_S;
    }
    else
    {
        speed.spin_rad_s = 0.0f;
    }
    if (mecanum_solve(&mecanum, &speed, wheel_rad_s) == ERR_OK)
    {
        for (uint32_t wheel_index = 0; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
        {
            (void)motor_set_target(&chassis_wheel[wheel_index]->base, wheel_rad_s[wheel_index] * M3508_GEAR_RATIO);
        }
    }
    (void)motor_compute_bus(CAN_BUS1);
#if defined(CONFIG_COMPONENT_POWER)
    chassis_power_apply();
#endif
    (void)motor_send_bus(CAN_BUS1);
#if defined(CONFIG_CDD_REFEREE) && defined(CONFIG_CDD_SUPERCAP)
    {
        static referee_info_t referee;

        referee_get(&referee);
        if (referee.online && referee.performance.chassis_power_limit != 0u)
        {
            supercap_set_referee(referee.performance.chassis_power_limit, referee.power_heat.buffer_energy);
        }
    }
    (void)supercap_send();
#elif defined(CONFIG_CDD_SUPERCAP)
    (void)supercap_send();
#endif
}
PERIOD_TASK_DEFINE(chassis_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, chassis_ctrl_run, NULL);
