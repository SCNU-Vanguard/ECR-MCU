/* 底盘：CAN1 四个 m3508，全向轮，速度环。速度读 operate，前后左右是云台坐标系。
 * 导航打开时改跟上位机速度；这一拍没有速度就停，不用摇杆顶上。
 * yaw 电机多圈角度把它们旋到车体坐标。失能时输出关掉。
 * 功率只限制这四个轮。超电在 CAN2。 */
#include <math.h>
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/operate/operate.h"
#include "cdd/motor/motor_init.h"
#include "cdd/supercap/supercap.h"
#include "cdd/referee/referee.h"
#include "cdd/host/host.h"
#include "component/power/power.h"
#include "chassis_config.h"
#include "supercap_config.h"
#include "power_config.h"
#include "host_config.h"

static dji_motor_t *const chassis_wheel[CHASSIS_WHEEL_COUNT] =
{
    &chassis_m3508_0,
    &chassis_m3508_1,
    &chassis_m3508_2,
    &chassis_m3508_3,
};

static const power_model_t power_model = POWER_MODEL_INIT;
static bool wheels_on;

static void wheels_set_active(bool active)
{
    for (uint32_t wheel_index = 0; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
    {
        motor_set_active(&chassis_wheel[wheel_index]->base, active);
    }
    wheels_on = active;
}

static void chassis_command(bool nav, float *forward_m_s, float *right_m_s, float *spin_rad_s)
{
    *forward_m_s = 0.0f;
    *right_m_s = 0.0f;
    *spin_rad_s = 0.0f;
    (void)operate_get(OPERATE_ID_FORWARD, forward_m_s, sizeof *forward_m_s);
    (void)operate_get(OPERATE_ID_RIGHT, right_m_s, sizeof *right_m_s);
    (void)operate_get(OPERATE_ID_SPIN, spin_rad_s, sizeof *spin_rad_s);
    if (nav)
    {
        *forward_m_s = 0.0f;
        *right_m_s = 0.0f;
        *spin_rad_s = 0.0f;
#ifdef CONFIG_CDD_HOST_NAV
        {
            host_nav_speed_t host_nav;

            if (host_transfer(HOST_CMD_NAV_SPEED, &host_nav, sizeof host_nav) == ERR_OK)
            {
                *forward_m_s = host_nav.forward_m_s;
                *right_m_s = host_nav.right_m_s;
                *spin_rad_s = host_nav.spin_rad_s;
            }
        }
#endif
    }
}

static float chassis_power_limit_w(void)
{
    power_budget_t budget = {0};

    static referee_info_t referee;

    referee_get(&referee);
    if (referee.online && referee.performance.chassis_power_limit != 0u)
    {
        budget.referee_valid = true;
        budget.referee_limit_w = (float)referee.performance.chassis_power_limit;
    }

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
    return power_budget_w(&budget);
}

static void chassis_power_apply(void)
{
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

/* 云台坐标系到车体坐标系。yaw_rad 是 yaw 电机多圈角，俯视逆时针为正，0 表示两套正前方重合。 */
static void gimbal_to_chassis(float yaw_rad, float forward_m_s, float right_m_s, chassis_speed_t *speed)
{
    float cosine = cosf(yaw_rad);
    float sine = sinf(yaw_rad);

    speed->forward_m_s = forward_m_s * cosine + right_m_s * sine;
    speed->right_m_s = -forward_m_s * sine + right_m_s * cosine;
}

static int32_t chassis_ctrl_init(void)
{
    supercap_enable(true);
    supercap_set_referee(SUPERCAP_POWER_W, SUPERCAP_BUFFER_J);
    return ERR_OK;
}
INIT_APP_STAGE2(chassis_ctrl_init);

static void chassis_ctrl_run(void *argument)
{
    chassis_speed_t speed;
    float wheel_rad_s[CHASSIS_WHEEL_COUNT];
    float forward_m_s;
    float right_m_s;
    float spin_rad_s;
    bool enabled = false;
    bool nav = false;

    (void)argument;
    (void)operate_get(OPERATE_ID_ENABLE, &enabled, sizeof enabled);
    (void)operate_get(OPERATE_ID_NAV, &nav, sizeof nav);
    if (enabled)
    {
        chassis_command(nav, &forward_m_s, &right_m_s, &spin_rad_s);
        gimbal_to_chassis(yaw_j4310.base.total_angle_rad, forward_m_s, right_m_s, &speed);
        speed.spin_rad_s = spin_rad_s;
        if (!wheels_on)
        {
            wheels_set_active(true);
        }
        if (omni_solve(&omni, &speed, wheel_rad_s) == ERR_OK)
        {
            for (uint32_t wheel_index = 0; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
            {
                (void)motor_set_target(&chassis_wheel[wheel_index]->base, wheel_rad_s[wheel_index] * M3508_GEAR_RATIO);
            }
        }
    }
    else if (wheels_on)
    {
        wheels_set_active(false);
    }
    (void)motor_compute_bus(CAN_BUS1);
    if (enabled)
    {
        chassis_power_apply();
    }
    (void)motor_send_bus(CAN_BUS1);

    {
        static referee_info_t referee;

        referee_get(&referee);
        if (referee.online && referee.performance.chassis_power_limit != 0u)
        {
            supercap_set_referee(referee.performance.chassis_power_limit, referee.power_heat.buffer_energy);
        }
    }
    (void)supercap_send();
}
PERIOD_TASK_DEFINE(chassis_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, chassis_ctrl_run, NULL);
