/* 云台板：pitch 用本板 IMU 做角度环。yaw 的当前角和目标角发给底盘，本板不转 yaw 电机。
 * 左拨杆上且视觉锁定时，pitch/yaw 目标改跟自瞄；没锁定就用手。下拨杆整机失能，电机输出关掉。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/remote/dt7/dt7.h"
#include "cdd/imu/ins/ins.h"
#include "cdd/motor/motor_init.h"
#include "cdd/host/host.h"
#include "cdd/interboard/interboard.h"
#include "interboard_config.h"
#include "host_config.h"

#define GIMBAL_DT_S              0.001f
#define GIMBAL_PITCH_LIMIT_RAD   0.45f
#define DT7_SWITCH_UP            1u
#define DT7_SWITCH_MID           3u
#define STICK_MAX                660.0f
#define STICK_IGNORE             20
#define DIAL_IGNORE              100
#define SPEED_MAX_M_S            1.5f
#define SPIN_RAD_S               2.0f
#define YAW_RATE_RAD_S           2.0f
#define PITCH_RATE_RAD_S         1.0f

static ins_attitude_t gimbal_attitude;
static bool pitch_on;
static bool tracking;
static float pitch_target;
static float yaw_target;

static float stick_to_speed(int16_t raw)
{
    if (raw > -STICK_IGNORE && raw < STICK_IGNORE)
    {
        return 0.0f;
    }
    return (float)raw / STICK_MAX * SPEED_MAX_M_S;
}

static float stick_to_rate(int16_t raw, float max_rad_s)
{
    if (raw > -STICK_IGNORE && raw < STICK_IGNORE)
    {
        return 0.0f;
    }
    return (float)raw / STICK_MAX * max_rad_s;
}

static float clamp_pitch(float angle)
{
    if (angle > GIMBAL_PITCH_LIMIT_RAD)
    {
        return GIMBAL_PITCH_LIMIT_RAD;
    }
    if (angle < -GIMBAL_PITCH_LIMIT_RAD)
    {
        return -GIMBAL_PITCH_LIMIT_RAD;
    }
    return angle;
}

static bool vision_locked(host_aim_target_t *aim)
{
    if (host_transfer(HOST_CMD_AIM_TARGET, aim, sizeof *aim) != ERR_OK)
    {
        return false;
    }
    return aim->is_locked != 0u;
}

static int32_t gimbal_ctrl_init(void)
{
    motor_bind_angle_feedback(&pitch_gm6020.base, &gimbal_attitude.pitch_rad);
    motor_bind_speed_feedback(&pitch_gm6020.base, &gimbal_attitude.gyro_rad_s[1]);
    return ERR_OK;
}
INIT_APP_STAGE2(gimbal_ctrl_init);

static void gimbal_ctrl_run(void *argument)
{
    dt7_data_t remote;
    host_aim_target_t aim;
    bool enabled = false;
    bool vision = false;
    bool trigger_on = false;
    float forward_m_s = 0.0f;
    float right_m_s = 0.0f;
    float spin_rad_s = 0.0f;
    float yaw_rate_rad_s = 0.0f;
    float pitch_rate_rad_s = 0.0f;
    bool locked;
    bool usable;

    (void)argument;
    ins_get_attitude(&gimbal_attitude);
    dt7_get(&remote);
    if (remote.online && remote.switch_left == DT7_SWITCH_UP)
    {
        enabled = true;
        vision = true;
    }
    else if (remote.online && remote.switch_left == DT7_SWITCH_MID)
    {
        enabled = true;
    }
    if (enabled)
    {
        trigger_on = remote.switch_right == DT7_SWITCH_UP;
        forward_m_s = stick_to_speed(remote.channel[3]);
        right_m_s = stick_to_speed(remote.channel[2]);
        if (remote.wheel > DIAL_IGNORE)
        {
            spin_rad_s = SPIN_RAD_S;
        }
        else if (remote.wheel < -DIAL_IGNORE)
        {
            spin_rad_s = -SPIN_RAD_S;
        }
        yaw_rate_rad_s = stick_to_rate(remote.channel[0], -YAW_RATE_RAD_S);
        pitch_rate_rad_s = stick_to_rate(remote.channel[1], PITCH_RATE_RAD_S);
    }
    locked = vision && vision_locked(&aim);
    usable = enabled && gimbal_attitude.valid;

    if (usable)
    {
        if (locked)
        {
            pitch_target = clamp_pitch(aim.pitch_rad);
            yaw_target = aim.yaw_rad;
        }
        else if (!pitch_on || tracking)
        {
            pitch_target = gimbal_attitude.pitch_rad;
            yaw_target = gimbal_attitude.yaw_rad;
        }
        else
        {
            pitch_target = clamp_pitch(pitch_target + pitch_rate_rad_s * GIMBAL_DT_S);
            yaw_target += yaw_rate_rad_s * GIMBAL_DT_S;
        }
        if (!pitch_on)
        {
            motor_set_active(&pitch_gm6020.base, true);
            pitch_on = true;
        }
        (void)motor_set_target(&pitch_gm6020.base, pitch_target);
    }
    else if (pitch_on)
    {
        motor_set_active(&pitch_gm6020.base, false);
        pitch_on = false;
    }
    tracking = locked && usable;

    interboard_request_t request;

    interboard_load_request(&request);
    request.motion.forward_m_s = forward_m_s;
    request.motion.right_m_s = right_m_s;
    request.motion.spin_rad_s = spin_rad_s;
    request.motion.yaw_rad = gimbal_attitude.yaw_rad;
    request.motion.yaw_target_rad = yaw_target;
    request.motion.enabled = enabled ? 1u : 0u;
    request.motion.yaw_valid = gimbal_attitude.valid ? 1u : 0u;
    request.shoot.trigger_on = trigger_on ? 1u : 0u;
    interboard_store_request(&request);
    (void)motor_compute_bus(CAN_BUS1);
    (void)motor_send_bus(CAN_BUS1);
}
PERIOD_TASK_DEFINE(gimbal_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, gimbal_ctrl_run, NULL);
