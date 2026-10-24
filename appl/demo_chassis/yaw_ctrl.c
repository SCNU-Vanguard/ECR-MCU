/* yaw：j4310，CAN2。角度当前值和目标都用云台 IMU 经板间传来的数。
 * 速度环仍用电机自己的转速。四个轮子的转速做全向轮逆解算，得到底盘自旋，取负号作为速度前馈，
 * 让电机跟着底盘反转，云台绝对角不跟底盘一起转。
 * 电机正转和 IMU yaw 同号（reverse = false）。底盘俯视逆时针为正，所以前馈是 -spin。
 * 板间离线、整机失能或云台 IMU 还无效时，输出关掉。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/motor/motor_init.h"
#include "cdd/interboard/interboard.h"
#include "interboard_config.h"
#include "chassis_config.h"

static float yaw_measure;
static float yaw_rate_ff;
static bool yaw_on;

static bool wheel_rates(float wheel_rad_s[CHASSIS_WHEEL_COUNT])
{
    dji_motor_t *const wheel[CHASSIS_WHEEL_COUNT] =
    {
        &chassis_m3508_0,
        &chassis_m3508_1,
        &chassis_m3508_2,
        &chassis_m3508_3,
    };

    for (uint32_t wheel_index = 0; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
    {
        if (!motor_online(&wheel[wheel_index]->base))
        {
            return false;
        }
        wheel_rad_s[wheel_index] = wheel[wheel_index]->base.speed_rad_s / M3508_GEAR_RATIO;
    }
    return true;
}

static int32_t yaw_ctrl_init(void)
{
    motor_bind_angle_feedback(&yaw_j4310.base, &yaw_measure);
    motor_bind_speed_feedforward(&yaw_j4310.base, &yaw_rate_ff);
    return dm_motor_enable(&yaw_j4310);
}
INIT_APP_STAGE2(yaw_ctrl_init);

static void yaw_ctrl_run(void *argument)
{
    interboard_link_t link;
    interboard_request_t request;
    float wheel_rad_s[CHASSIS_WHEEL_COUNT];
    chassis_speed_t speed;
    bool running;

    (void)argument;
    interboard_get_link(&link);
    running = link.online && interboard_load_request(&request) &&
              request.motion.enabled != 0u && request.motion.yaw_valid != 0u;
    yaw_rate_ff = 0.0f;
    if (running)
    {
        yaw_measure = request.motion.yaw_rad;
        if (wheel_rates(wheel_rad_s) && omni_inverse(&omni, wheel_rad_s, &speed) == ERR_OK)
        {
            yaw_rate_ff = -speed.spin_rad_s;
        }
        if (!yaw_on)
        {
            motor_set_active(&yaw_j4310.base, true);
            yaw_on = true;
        }
        (void)motor_set_target(&yaw_j4310.base, request.motion.yaw_target_rad);
    }
    else if (yaw_on)
    {
        motor_set_active(&yaw_j4310.base, false);
        yaw_on = false;
    }
    (void)motor_compute_bus(CAN_BUS2);
    (void)motor_send_bus(CAN_BUS2);
}
PERIOD_TASK_DEFINE(yaw_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, yaw_ctrl_run, NULL);
