/* 云台：CAN2，pitch = gm6020，yaw = j4310。角度和转速都用电机自己的反馈。
 * 四个轮速做全向轮逆解算，自旋取负号作为 yaw 速度前馈。
 * 自瞄打开、视觉锁定且给出的 pitch/yaw 不全为 0 时跟自瞄，否则用手。
 * 手控是 operate 给出的目标转速，每个任务周期加到电机目标角上。
 * 失能时两台一起关掉。打开后各发控制帧，谁收到反馈谁先对齐再积分，互不等待。
 * 一台没反馈或掉线时，另一台照常跟手。
 * IMU 只上报给上位机，不参与这两个环。 */
#include <string.h>
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/operate/operate.h"
#include "cdd/imu/ins/ins.h"
#include "cdd/motor/motor_init.h"
#include "cdd/host/host.h"
#include "cdd/referee/referee.h"
#include "component/chassis/omni/omni.h"
#include "host_config.h"
#include "chassis_config.h"

#define GIMBAL_PITCH_MAX_RAD       0.45f    /* pitch 目标角上限，相对电机零点 */
#define GIMBAL_PITCH_MIN_RAD      -0.45f    /* pitch 目标角下限，和上限不必对称 */
#define GIMBAL_CTRL_PERIOD_S      (1.0f / (float)OS_FREQ_1000HZ) /* 与 gimbal_ctrl_task 同周期 */

static const omni_geometry_t chassis_geometry = CHASSIS_GEOMETRY_INIT;
static ins_attitude_t gimbal_attitude;
static bool gimbal_on;      /* 操作手使能后两台输出都开着。关的是一起关，动起来各看各的反馈 */
static bool pitch_aligned;  /* pitch 目标已对齐电机角度。没对齐之前不积分 */
static bool yaw_aligned;
static bool was_locked;     /* 上一拍自瞄锁定。刚退出时各自先对齐当前角 */
static float pitch_target;
static float yaw_target;
static float yaw_rate_ff;

static float clamp_pitch(float angle)
{
    if (angle > GIMBAL_PITCH_MAX_RAD)
    {
        return GIMBAL_PITCH_MAX_RAD;
    }
    if (angle < GIMBAL_PITCH_MIN_RAD)
    {
        return GIMBAL_PITCH_MIN_RAD;
    }
    return angle;
}

/* 取自瞄目标。视觉已锁定且 pitch、yaw 不全为 0 才返回 true。 */
static bool aim_target_get(float *pitch_rad, float *yaw_rad)
{
    host_aim_target_t aim;

    if (!host_load(HOST_CMD_AIM_TARGET, &aim, sizeof aim))
    {
        return false;
    }
    if (aim.is_locked == 0u || (aim.pitch_rad == 0.0f && aim.yaw_rad == 0.0f))
    {
        return false;
    }
    *pitch_rad = aim.pitch_rad;
    *yaw_rad = aim.yaw_rad;
    return true;
}

static void host_cache_store(void)
{
    host_aim_attitude_t current;
    host_nav_state_t nav = {0};
    referee_info_t referee;

    if (!gimbal_attitude.valid)
    {
        return;
    }
    current.roll_rad = gimbal_attitude.roll_rad;
    current.pitch_rad = gimbal_attitude.pitch_rad;
    current.yaw_rad = gimbal_attitude.yaw_rad;
    host_store(HOST_CMD_AIM_ATTITUDE, &current, sizeof current);

    referee_get(&referee);
    nav.hp = referee.performance.current_hp;
    nav.roll_rad = gimbal_attitude.roll_rad;
    nav.pitch_rad = gimbal_attitude.pitch_rad;
    nav.yaw_rad = gimbal_attitude.yaw_rad;
    host_store(HOST_CMD_NAV_STATE, &nav, sizeof nav);
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
        wheel_rad_s[wheel_index] = wheel[wheel_index]->base.speed_rad_s / CHASSIS_DRIVE_RATIO;
    }
    return true;
}

static void gimbal_set_active(bool active)
{
    motor_set_active(&pitch_gm6020.base, active);
    motor_set_active(&yaw_j4310.base, active);
    gimbal_on = active;
}

/* 一台的目标角，和另一台无关。还没收到过反馈就返回 false，本拍不写这台的目标。
 * 刚对齐或刚退出自瞄：目标等于当前角。自瞄中用给定角。手控且在线：转速乘周期累加。掉线则保持。 */
static bool axis_follow(motor_t *motor, bool *aligned, float *target,
                        bool locked, float aim_rad, float rate, bool pitch_limit)
{
    if (!*aligned)
    {
        if (!motor->got_first)
        {
            return false;
        }
        *target = motor->single_angle_rad;
        *aligned = true;
    }
    else if (locked)
    {
        *target = aim_rad;
    }
    else if (was_locked)
    {
        *target = motor->single_angle_rad;
    }
    else if (motor_online(motor))
    {
        *target += rate * GIMBAL_CTRL_PERIOD_S;
    }
    if (pitch_limit)
    {
        *target = clamp_pitch(*target);
    }
    return true;
}

static void yaw_spin_ff(void)
{
    float wheel_rad_s[CHASSIS_WHEEL_COUNT];
    chassis_speed_t speed;

    yaw_rate_ff = 0.0f;
    if (wheel_rates(wheel_rad_s) && omni_inverse(&chassis_geometry, wheel_rad_s, &speed) == ERR_OK)
    {
        yaw_rate_ff = -speed.spin_rad_s;
    }
}

static int32_t gimbal_ctrl_init(void)
{
    motor_bind_speed_feedforward(&yaw_j4310.base, &yaw_rate_ff);
    return dm_motor_enable(&yaw_j4310);
}
INIT_APP_STAGE2(gimbal_ctrl_init);

static void gimbal_ctrl_run(void *argument)
{
    operate_snap_t op;
    float aim_pitch_rad = 0.0f;
    float aim_yaw_rad = 0.0f;
    bool locked;

    (void)argument;
    ins_get_attitude(&gimbal_attitude);
    host_cache_store();
    /* 本拍只取一次，转速和开关来自同一次发布。 */
    if (operate_get_snapshot(&op) != ERR_OK)
    {
        memset(&op, 0, sizeof op);
    }
    locked = op.use_aim && aim_target_get(&aim_pitch_rad, &aim_yaw_rad);

    if (!op.enabled)
    {
        if (gimbal_on)
        {
            gimbal_set_active(false);
        }
        pitch_aligned = false;
        yaw_aligned = false;
        was_locked = false;
        yaw_rate_ff = 0.0f;
    }
    else
    {
        if (!gimbal_on)
        {
            gimbal_set_active(true);
        }
        if (axis_follow(&pitch_gm6020.base, &pitch_aligned, &pitch_target,
                        locked, aim_pitch_rad, op.pitch_rate_rad_s, true))
        {
            (void)motor_set_target(&pitch_gm6020.base, pitch_target);
        }
        if (axis_follow(&yaw_j4310.base, &yaw_aligned, &yaw_target,
                        locked, aim_yaw_rad, op.yaw_rate_rad_s, false))
        {
            (void)motor_set_target(&yaw_j4310.base, yaw_target);
        }
        was_locked = locked && (pitch_aligned || yaw_aligned);
        yaw_spin_ff();
    }

    (void)motor_compute_bus(CAN_BUS2);
    (void)motor_send_bus(CAN_BUS2);
}
PERIOD_TASK_DEFINE(gimbal_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, gimbal_ctrl_run, NULL);
