/* 云台板：pitch 用本板 IMU 做角度环。本板不转 yaw 电机。
 * 自瞄打开且视觉锁定时跟自瞄，否则用手。手控增量来自 operate，seq 变了才加一次。
 * 失能或 IMU 无效时 pitch 输出关掉。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/operate/operate.h"
#include "cdd/imu/ins/ins.h"
#include "cdd/motor/motor_init.h"
#include "cdd/host/host.h"
#include "host_config.h"

#define GIMBAL_PITCH_LIMIT_RAD   0.45f

static ins_attitude_t gimbal_attitude;
static bool pitch_on;   /* 电机输出开着。关着的下一拍先对齐当前角 */
static bool was_locked;  /* 上一拍自瞄锁定。刚退出时先对齐当前角 */
static float pitch_target;

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
    host_aim_target_t aim;
    static uint32_t applied_seq;
    float pitch_delta_rad = 0.0f;
    uint32_t seq = 0u;
    bool enabled = false;
    bool use_aim = false;
    bool locked;
    bool usable;

    (void)argument;
    ins_get_attitude(&gimbal_attitude);
    (void)operate_get(OPERATE_ID_ENABLE, &enabled, sizeof enabled);
    (void)operate_get(OPERATE_ID_AIM, &use_aim, sizeof use_aim);
    (void)operate_get(OPERATE_ID_SEQ, &seq, sizeof seq);
    if (seq != applied_seq)
    {
        (void)operate_get(OPERATE_ID_PITCH_DELTA, &pitch_delta_rad, sizeof pitch_delta_rad);
        applied_seq = seq;
    }
    locked = use_aim && vision_locked(&aim);
    usable = enabled && gimbal_attitude.valid;

    if (usable)
    {
        if (locked)
        {
            pitch_target = clamp_pitch(aim.pitch_rad);
        }
        else if (!pitch_on || was_locked)
        {
            pitch_target = gimbal_attitude.pitch_rad;
        }
        else
        {
            pitch_target = clamp_pitch(pitch_target + pitch_delta_rad);
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
    was_locked = locked && usable;
    (void)motor_compute_bus(CAN_BUS1);
    (void)motor_send_bus(CAN_BUS1);
}
PERIOD_TASK_DEFINE(gimbal_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, gimbal_ctrl_run, NULL);
