/* 底盘：CAN1 四个 m3508，速度环。构型、几何、减速比和功率模型在 chassis_config.h。
 * 速度读 operate，前后左右是云台坐标系。
 * 上位机给出非零速度时改跟上位机速度；否则仍用手动速度。
 * yaw 电机多圈角度把它们旋到车体坐标。失能时输出关掉。
 * 功率只限制这四个驱动轮。超电在 CAN2。 */
#include <math.h>
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/operate/operate.h"
#include "cdd/motor/motor_init.h"
#include "cdd/supercap/supercap.h"
#include "cdd/referee/referee.h"
#include "cdd/host/host.h"
#include "component/chassis/chassis.h"
#include "supercap_config.h"
#include "host_config.h"

static motor_t *const chassis_wheel[CHASSIS_WHEEL_COUNT] =
{
    &chassis_m3508_0.base,
    &chassis_m3508_1.base,
    &chassis_m3508_2.base,
    &chassis_m3508_3.base,
};

static referee_info_t referee;   /* 每个控制周期只取一次，功率限制和超电同步共用 */
static bool wheels_on;

static void wheels_set_active(bool active)
{
    for (uint32_t wheel_index = 0; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
    {
        motor_set_active(chassis_wheel[wheel_index], active);
    }
    wheels_on = active;
}

/* 默认用手动速度。上位机给出了非零速度时才换成上位机速度。 */
static void chassis_command(const operate_snap_t *op, float *forward_m_s, float *right_m_s, float *spin_rad_s)
{
    host_nav_speed_t host_nav;

    *forward_m_s = op->forward_m_s;
    *right_m_s = op->right_m_s;
    *spin_rad_s = op->spin_rad_s;
    if (op->use_nav &&
        host_load(HOST_CMD_NAV_SPEED, &host_nav, sizeof host_nav) &&
        (host_nav.forward_m_s != 0.0f || host_nav.right_m_s != 0.0f || host_nav.spin_rad_s != 0.0f))
    {
        *forward_m_s = host_nav.forward_m_s;
        *right_m_s = host_nav.right_m_s;
        *spin_rad_s = host_nav.spin_rad_s;
    }
}

static float chassis_power_limit_w(void)
{
    chassis_power_source_t source = {0};
    supercap_status_t supercap;

    if (referee.online && referee.performance.chassis_power_limit != 0u)
    {
        source.referee_valid = true;
        source.referee_limit_w = (float)referee.performance.chassis_power_limit;
    }

    supercap_get(&supercap);
    if (supercap.online && supercap.error != SUPERCAP_ERROR_FATAL)
    {
        source.supercap_valid = true;
        source.energy_raw = supercap.energy_raw;
        source.supercap_limit_w = (float)supercap.chassis_power_limit_w;
    }
    return chassis_power_budget_w(&source);
}

/* 裁判系统功率上限和缓冲能量只在变化时同步给超电。 */
static void supercap_referee_sync(void)
{
    static uint32_t last_limit_w;
    static uint32_t last_buffer_j;
    uint32_t limit_w = referee.performance.chassis_power_limit;
    uint32_t buffer_j = referee.power_heat.buffer_energy;

    if (!referee.online || limit_w == 0u)
    {
        return;
    }
    if (limit_w != last_limit_w || buffer_j != last_buffer_j)
    {
        supercap_set_referee(limit_w, buffer_j);
        last_limit_w = limit_w;
        last_buffer_j = buffer_j;
    }
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
    float forward_m_s;
    float right_m_s;
    float spin_rad_s;
    operate_snap_t op = {0};
    bool enabled;

    (void)argument;
    (void)operate_get_snapshot(&op);
    referee_get(&referee);
    enabled = op.enabled;
    if (enabled)
    {
        chassis_command(&op, &forward_m_s, &right_m_s, &spin_rad_s);
        gimbal_to_chassis(yaw_j4310.base.total_angle_rad, forward_m_s, right_m_s, &speed);
        speed.spin_rad_s = spin_rad_s;
        if (!wheels_on)
        {
            wheels_set_active(true);
        }
        (void)chassis_solve(chassis_wheel, &speed);
    }
    else if (wheels_on)
    {
        wheels_set_active(false);
    }
    (void)motor_compute_bus(CAN_BUS1);
    if (enabled)
    {
        (void)chassis_power_apply(chassis_wheel, CHASSIS_WHEEL_COUNT, chassis_power_limit_w());
    }
    (void)motor_send_bus(CAN_BUS1);

    supercap_referee_sync();
    (void)supercap_send();
}
PERIOD_TASK_DEFINE(chassis_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, chassis_ctrl_run, NULL);
