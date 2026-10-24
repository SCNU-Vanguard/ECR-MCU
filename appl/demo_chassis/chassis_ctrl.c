/* 底盘：CAN1 四个 m3508，全向轮，速度环。速度来自云台板，前后左右是云台坐标系。
 * yaw 电机多圈角度把它们旋到车体坐标，自旋不转。失能或板间离线时输出关掉。
 * 功率只限制这四个轮。超电在 CAN2，裁判功率上限从本板裁判系统写入。
 * 应答带回车体速度、yaw 电机角度、17 mm 枪口热量和热量上限。 */
#include <math.h>
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/motor/motor_init.h"
#include "cdd/supercap/supercap.h"
#include "cdd/referee/referee.h"
#include "cdd/interboard/interboard.h"
#include "component/power/power.h"
#include "interboard_config.h"
#include "chassis_config.h"
#include "supercap_config.h"
#include "power_config.h"

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

static bool chassis_measured(chassis_speed_t *speed)
{
    float wheel_rad_s[CHASSIS_WHEEL_COUNT];

    for (uint32_t wheel_index = 0; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
    {
        if (!motor_online(&chassis_wheel[wheel_index]->base))
        {
            return false;
        }
        wheel_rad_s[wheel_index] = chassis_wheel[wheel_index]->base.speed_rad_s / M3508_GEAR_RATIO;
    }
    return omni_inverse(&omni, wheel_rad_s, speed) == ERR_OK;
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
    interboard_link_t link;
    interboard_request_t request;
    interboard_reply_t reply;
    chassis_speed_t speed;
    chassis_speed_t measured;
    float wheel_rad_s[CHASSIS_WHEEL_COUNT];
    float yaw_motor_rad;
    bool running;

    (void)argument;
    yaw_motor_rad = yaw_j4310.base.total_angle_rad;
    interboard_get_link(&link);
    running = link.online && interboard_load_request(&request) && request.motion.enabled != 0u;
    speed.forward_m_s = 0.0f;
    speed.right_m_s = 0.0f;
    speed.spin_rad_s = 0.0f;
    if (running)
    {
        gimbal_to_chassis(yaw_motor_rad, request.motion.forward_m_s, request.motion.right_m_s, &speed);
        speed.spin_rad_s = request.motion.spin_rad_s;
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
    if (running)
    {
        chassis_power_apply();
    }
    (void)motor_send_bus(CAN_BUS1);

    interboard_load_reply(&reply);
    {
        static referee_info_t referee;

        referee_get(&referee);
        reply.heat = referee.power_heat.shooter_17mm_barrel_heat;
        reply.heat_limit = referee.performance.shooter_barrel_heat_limit;
        if (referee.online && referee.performance.chassis_power_limit != 0u)
        {
            supercap_set_referee(referee.performance.chassis_power_limit, referee.power_heat.buffer_energy);
        }
    }
    reply.yaw_motor_rad = yaw_motor_rad;
    if (chassis_measured(&measured))
    {
        reply.forward_m_s = measured.forward_m_s;
        reply.right_m_s = measured.right_m_s;
        reply.spin_rad_s = measured.spin_rad_s;
    }
    else
    {
        reply.forward_m_s = speed.forward_m_s;
        reply.right_m_s = speed.right_m_s;
        reply.spin_rad_s = speed.spin_rad_s;
    }
    interboard_store_reply(&reply);
    (void)supercap_send();
}
PERIOD_TASK_DEFINE(chassis_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, chassis_ctrl_run, NULL);
