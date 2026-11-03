/* 板间主机：1000 Hz 组请求并发出。
 * 速度来自 operate。导航打开时改跟上位机，这一拍没有速度就停。
 * yaw 当前角来自本板 IMU。目标角在这里自己积：自瞄锁定时跟视觉，否则用手，seq 变了才加一次。
 * 发射要整车使能且摩擦轮开着。 */
#include "osal/period.h"
#include "component/common/err.h"
#include "cdd/operate/operate.h"
#include "cdd/imu/ins/ins.h"
#include "cdd/host/host.h"
#include "cdd/interboard/interboard.h"
#include "host_config.h"

static bool yaw_started;  /* 这一次使能里已经对齐过当前角 */
static bool was_locked;    /* 上一拍自瞄锁定。刚退出时先对齐当前角 */
static float yaw_target;

static bool vision_locked(host_aim_target_t *aim)
{
    if (host_transfer(HOST_CMD_AIM_TARGET, aim, sizeof *aim) != ERR_OK)
    {
        return false;
    }
    return aim->is_locked != 0u;
}

/* 刚使能、或刚退出自瞄时先对齐当前角，之后只加增量。 */
static float yaw_command(bool usable, bool locked, float yaw_rad, float aim_yaw_rad, float yaw_delta_rad)
{
    if (usable)
    {
        if (locked)
        {
            yaw_target = aim_yaw_rad;
        }
        else if (!yaw_started || was_locked)
        {
            yaw_target = yaw_rad;
        }
        else
        {
            yaw_target += yaw_delta_rad;
        }
        yaw_started = true;
    }
    else
    {
        yaw_started = false;
    }
    was_locked = locked && usable;
    return yaw_target;
}

static void interboard_ctrl_run(void *argument)
{
    ins_attitude_t attitude;
    host_aim_target_t aim = {0};
    interboard_request_t request;
    static uint32_t applied_seq;
    float forward_m_s = 0.0f;
    float right_m_s = 0.0f;
    float spin_rad_s = 0.0f;
    float yaw_delta_rad = 0.0f;
    uint32_t seq = 0u;
    bool enabled = false;
    bool nav = false;
    bool use_aim = false;
    bool friction = false;
    bool shoot = false;
    bool locked;
    bool usable;

    (void)argument;
    ins_get_attitude(&attitude);
    (void)operate_get(OPERATE_ID_ENABLE, &enabled, sizeof enabled);
    (void)operate_get(OPERATE_ID_NAV, &nav, sizeof nav);
    (void)operate_get(OPERATE_ID_AIM, &use_aim, sizeof use_aim);
    (void)operate_get(OPERATE_ID_FRICTION, &friction, sizeof friction);
    (void)operate_get(OPERATE_ID_SHOOT, &shoot, sizeof shoot);
    (void)operate_get(OPERATE_ID_SEQ, &seq, sizeof seq);
    if (seq != applied_seq)
    {
        (void)operate_get(OPERATE_ID_YAW_DELTA, &yaw_delta_rad, sizeof yaw_delta_rad);
        applied_seq = seq;
    }
    locked = use_aim && vision_locked(&aim);
    usable = enabled && attitude.valid;
    if (enabled)
    {
        (void)operate_get(OPERATE_ID_FORWARD, &forward_m_s, sizeof forward_m_s);
        (void)operate_get(OPERATE_ID_RIGHT, &right_m_s, sizeof right_m_s);
        (void)operate_get(OPERATE_ID_SPIN, &spin_rad_s, sizeof spin_rad_s);
        if (nav)
        {
            forward_m_s = 0.0f;
            right_m_s = 0.0f;
            spin_rad_s = 0.0f;
#ifdef CONFIG_CDD_HOST_NAV
            {
                host_nav_speed_t host_nav;

                if (host_transfer(HOST_CMD_NAV_SPEED, &host_nav, sizeof host_nav) == ERR_OK)
                {
                    forward_m_s = host_nav.forward_m_s;
                    right_m_s = host_nav.right_m_s;
                    spin_rad_s = host_nav.spin_rad_s;
                }
            }
#endif
        }
    }

    interboard_load_request(&request);
    request.motion.forward_m_s = forward_m_s;
    request.motion.right_m_s = right_m_s;
    request.motion.spin_rad_s = spin_rad_s;
    request.motion.yaw_rad = attitude.yaw_rad;
    request.motion.yaw_target_rad = yaw_command(usable, locked, attitude.yaw_rad, aim.yaw_rad, yaw_delta_rad);
    request.motion.enabled = enabled ? 1u : 0u;
    request.motion.yaw_valid = attitude.valid ? 1u : 0u;
    request.shoot.trigger_on = (enabled && friction && shoot) ? 1u : 0u;
    interboard_store_request(&request);
    (void)interboard_post();
}
PERIOD_TASK_DEFINE(interboard_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, interboard_ctrl_run, NULL);
