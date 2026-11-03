/* 云台：CAN2，pitch = gm6020，yaw = j4310。角度都用本板 IMU。
 * yaw 速度环仍用电机自己的转速。四个轮速做全向轮逆解算，自旋取负号作为速度前馈。
 * 自瞄打开且视觉锁定时跟自瞄，否则用手。手控增量来自 operate，seq 变了才加一次。
 * 失能或 IMU 无效时输出关掉。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/operate/operate.h"
#include "cdd/imu/ins/ins.h"
#include "cdd/motor/motor_init.h"
#include "cdd/host/host.h"
#include "host_config.h"
#include "chassis_config.h"

#define GIMBAL_PITCH_LIMIT_RAD   0.45f

static ins_attitude_t gimbal_attitude;
static bool pitch_on;   /* 电机输出开着。关着的下一拍先对齐当前角 */
static bool yaw_on;
static bool was_locked;  /* 上一拍自瞄锁定。刚退出时先对齐当前角 */
static float pitch_target;
static float yaw_target;
static float yaw_measure;
static float yaw_rate_ff;

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

static int32_t gimbal_ctrl_init(void)
{
    motor_bind_angle_feedback(&pitch_gm6020.base, &gimbal_attitude.pitch_rad);
    motor_bind_speed_feedback(&pitch_gm6020.base, &gimbal_attitude.gyro_rad_s[1]);
    motor_bind_angle_feedback(&yaw_j4310.base, &yaw_measure);
    motor_bind_speed_feedforward(&yaw_j4310.base, &yaw_rate_ff);
    return dm_motor_enable(&yaw_j4310);
}
INIT_APP_STAGE2(gimbal_ctrl_init);

static void gimbal_ctrl_run(void *argument)
{
    host_aim_target_t aim;
    float wheel_rad_s[CHASSIS_WHEEL_COUNT];
    chassis_speed_t speed;
    static uint32_t applied_seq;
    float yaw_delta_rad = 0.0f;
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
        (void)operate_get(OPERATE_ID_YAW_DELTA, &yaw_delta_rad, sizeof yaw_delta_rad);
        (void)operate_get(OPERATE_ID_PITCH_DELTA, &pitch_delta_rad, sizeof pitch_delta_rad);
        applied_seq = seq;
    }
    locked = use_aim && vision_locked(&aim);
    usable = enabled && gimbal_attitude.valid;

    yaw_rate_ff = 0.0f;
    if (usable)
    {
        if (locked)
        {
            pitch_target = clamp_pitch(aim.pitch_rad);
            yaw_target = aim.yaw_rad;
        }
        else if (!pitch_on || was_locked)
        {
            pitch_target = gimbal_attitude.pitch_rad;
            yaw_target = gimbal_attitude.yaw_rad;
        }
        else
        {
            pitch_target = clamp_pitch(pitch_target + pitch_delta_rad);
            yaw_target += yaw_delta_rad;
        }
        yaw_measure = gimbal_attitude.yaw_rad;
        if (wheel_rates(wheel_rad_s) && omni_inverse(&omni, wheel_rad_s, &speed) == ERR_OK)
        {
            yaw_rate_ff = -speed.spin_rad_s;
        }
        if (!pitch_on)
        {
            motor_set_active(&pitch_gm6020.base, true);
            pitch_on = true;
        }
        if (!yaw_on)
        {
            motor_set_active(&yaw_j4310.base, true);
            yaw_on = true;
        }
        (void)motor_set_target(&pitch_gm6020.base, pitch_target);
        (void)motor_set_target(&yaw_j4310.base, yaw_target);
    }
    else
    {
        if (pitch_on)
        {
            motor_set_active(&pitch_gm6020.base, false);
            pitch_on = false;
        }
        if (yaw_on)
        {
            motor_set_active(&yaw_j4310.base, false);
            yaw_on = false;
        }
    }
    was_locked = locked && usable;

    (void)motor_compute_bus(CAN_BUS2);
    (void)motor_send_bus(CAN_BUS2);
}
PERIOD_TASK_DEFINE(gimbal_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, gimbal_ctrl_run, NULL);
