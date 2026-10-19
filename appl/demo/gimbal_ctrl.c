/* 云台：CAN3，yaw = j4310，pitch = gm6020。右摇杆积分成角度目标。
 * 反馈在 INS 的 valid 变为 true 之后才绑定并打开输出，避免先按电机角吸合再切到陀螺仪。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/remote/dt7/dt7.h"
#include "cdd/imu/ins/ins.h"
#include "robot/demo/motor_config.h"

#define GIMBAL_STICK_MAX        660.0f
#define GIMBAL_STICK_IGNORE     20
#define GIMBAL_DT_S             0.001f
#define GIMBAL_YAW_RATE_RAD_S   2.0f
#define GIMBAL_PITCH_RATE_RAD_S 1.0f
#define GIMBAL_PITCH_LIMIT_RAD  0.45f

static ins_attitude_t gimbal_attitude;
static bool gimbal_feedback_ready;
static float gimbal_yaw_target;
static float gimbal_pitch_target;

static float stick_to_rate(int16_t raw, float max_rad_s)
{
    if (raw > -GIMBAL_STICK_IGNORE && raw < GIMBAL_STICK_IGNORE)
    {
        return 0.0f;
    }
    return (float)raw / GIMBAL_STICK_MAX * max_rad_s;
}

static int32_t gimbal_ctrl_init(void)
{
    return dm_motor_enable(&gimbal_j4310);
}
INIT_APP_STAGE2(gimbal_ctrl_init);

static void gimbal_bind_feedback(void)
{
    motor_bind_angle_feedback(&gimbal_j4310.base, &gimbal_attitude.yaw_rad);
    motor_bind_speed_feedback(&gimbal_j4310.base, &gimbal_attitude.gyro_rad_s[2]);
    motor_bind_angle_feedback(&gimbal_gm6020.base, &gimbal_attitude.pitch_rad);
    motor_bind_speed_feedback(&gimbal_gm6020.base, &gimbal_attitude.gyro_rad_s[1]);
    gimbal_yaw_target = gimbal_attitude.yaw_rad;
    gimbal_pitch_target = gimbal_attitude.pitch_rad;
    motor_set_active(&gimbal_j4310.base, true);
    motor_set_active(&gimbal_gm6020.base, true);
    gimbal_feedback_ready = true;
}

static void gimbal_ctrl_run(void *argument)
{
    dt7_data_t remote;

    (void)argument;
    ins_get_attitude(&gimbal_attitude);
    if (!gimbal_feedback_ready)
    {
        if (gimbal_attitude.valid)
        {
            gimbal_bind_feedback();
        }
    }
    else
    {
        dt7_get(&remote);
        gimbal_yaw_target += stick_to_rate(remote.channel[0], -GIMBAL_YAW_RATE_RAD_S) * GIMBAL_DT_S;
        gimbal_pitch_target += stick_to_rate(remote.channel[1], GIMBAL_PITCH_RATE_RAD_S) * GIMBAL_DT_S;
        if (gimbal_pitch_target > GIMBAL_PITCH_LIMIT_RAD)
        {
            gimbal_pitch_target = GIMBAL_PITCH_LIMIT_RAD;
        }
        else if (gimbal_pitch_target < -GIMBAL_PITCH_LIMIT_RAD)
        {
            gimbal_pitch_target = -GIMBAL_PITCH_LIMIT_RAD;
        }
        (void)motor_set_target(&gimbal_j4310.base, gimbal_yaw_target);
        (void)motor_set_target(&gimbal_gm6020.base, gimbal_pitch_target);
    }
    (void)motor_compute_bus(CAN_BUS2);
    (void)motor_send_bus(CAN_BUS2);
}
PERIOD_TASK_DEFINE(gimbal_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, gimbal_ctrl_run, NULL);
