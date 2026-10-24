/* 发射：CAN3。三个摩擦轮直接给原始输出，全正向，开时固定 6000。
 * 拨弹 m2006 速度环，只分转和不转，目标 40 rad/s。左拨杆下或离线时输出关掉。 */
#include "osal/period.h"
#include "cdd/remote/dt7/dt7.h"
#include "cdd/motor/motor_init.h"

#define DT7_SWITCH_UP         1u
#define DT7_SWITCH_MID        3u
#define SHOOT_FRICTION_CMD    6000
#define SHOOT_TRIGGER_RAD_S   40.0f

static bool trigger_active;

static void shoot_ctrl_run(void *argument)
{
    dt7_data_t remote;
    bool allowed;
    bool friction_on;
    bool spinning;
    int16_t friction_cmd;

    (void)argument;
    dt7_get(&remote);
    allowed = remote.online &&
              (remote.switch_left == DT7_SWITCH_UP || remote.switch_left == DT7_SWITCH_MID);
    friction_on = allowed &&
                  (remote.switch_right == DT7_SWITCH_UP || remote.switch_right == DT7_SWITCH_MID);
    spinning = allowed && remote.switch_right == DT7_SWITCH_UP;
    friction_cmd = friction_on ? SHOOT_FRICTION_CMD : 0;
    dji_motor_set_cmd(&shoot_fric_0, friction_cmd);
    dji_motor_set_cmd(&shoot_fric_1, friction_cmd);
    dji_motor_set_cmd(&shoot_fric_2, friction_cmd);
    if (allowed)
    {
        if (!trigger_active)
        {
            motor_set_active(&shoot_m2006.base, true);
            trigger_active = true;
        }
        (void)motor_set_target(&shoot_m2006.base, spinning ? SHOOT_TRIGGER_RAD_S : 0.0f);
    }
    else if (trigger_active)
    {
        motor_set_active(&shoot_m2006.base, false);
        trigger_active = false;
    }
    (void)motor_compute_bus(CAN_BUS3);
    (void)motor_send_bus(CAN_BUS3);
}
PERIOD_TASK_DEFINE(shoot_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, shoot_ctrl_run, NULL);
