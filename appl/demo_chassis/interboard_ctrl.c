/* 板间从机：1000 Hz 记下云台请求，并写回应答。
 * 底盘、yaw、发射从这里取请求。
 * 应答是车体速度、yaw 电机多圈角、17 mm 枪口热量和热量上限。
 * 车体速度优先用轮速逆解，逆解不了就用这一拍指令旋到车体坐标。 */
#include <math.h>
#include "osal/period.h"
#include "component/common/err.h"
#include "cdd/motor/motor_init.h"
#include "cdd/referee/referee.h"
#include "cdd/interboard/interboard.h"
#include "interboard_ctrl.h"
#include "chassis_config.h"

static interboard_request_t latest_request;
static bool latest_valid;

static dji_motor_t *const chassis_wheel[CHASSIS_WHEEL_COUNT] =
{
    &chassis_m3508_0,
    &chassis_m3508_1,
    &chassis_m3508_2,
    &chassis_m3508_3,
};

bool interboard_ctrl_get(interboard_request_t *request)
{
    uint32_t irq_key;
    bool ok;

    if (request == NULL)
    {
        return false;
    }
    irq_key = os_irq_save();
    ok = latest_valid;
    if (ok)
    {
        *request = latest_request;
    }
    os_irq_restore(irq_key);
    return ok;
}

/* yaw_rad 是 yaw 电机多圈角，俯视逆时针为正，0 表示两套正前方重合。 */
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

static void interboard_ctrl_run(void *argument)
{
    interboard_link_t link;
    interboard_request_t request;
    interboard_reply_t reply;
    chassis_speed_t measured;
    bool have_request;
    uint32_t irq_key;

    (void)argument;
    interboard_get_link(&link);
    have_request = link.online && interboard_load_request(&request);
    irq_key = os_irq_save();
    latest_valid = have_request;
    if (have_request)
    {
        latest_request = request;
    }
    os_irq_restore(irq_key);

    interboard_load_reply(&reply);
    reply.yaw_motor_rad = yaw_j4310.base.total_angle_rad;
    {
        static referee_info_t referee;

        referee_get(&referee);
        reply.heat = referee.power_heat.shooter_17mm_barrel_heat;
        reply.heat_limit = referee.performance.shooter_barrel_heat_limit;
    }
    if (chassis_measured(&measured))
    {
        reply.forward_m_s = measured.forward_m_s;
        reply.right_m_s = measured.right_m_s;
        reply.spin_rad_s = measured.spin_rad_s;
    }
    else if (have_request && request.motion.enabled != 0u)
    {
        chassis_speed_t speed;

        gimbal_to_chassis(reply.yaw_motor_rad, request.motion.forward_m_s, request.motion.right_m_s, &speed);
        reply.forward_m_s = speed.forward_m_s;
        reply.right_m_s = speed.right_m_s;
        reply.spin_rad_s = request.motion.spin_rad_s;
    }
    else
    {
        reply.forward_m_s = 0.0f;
        reply.right_m_s = 0.0f;
        reply.spin_rad_s = 0.0f;
    }
    interboard_store_reply(&reply);
}
PERIOD_TASK_DEFINE(interboard_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, interboard_ctrl_run, NULL);
