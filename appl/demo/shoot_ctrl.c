/* 发射：CAN3。三个摩擦轮直接给原始输出，全正向，开时固定 6000。
 * 拨弹 m2006 速度环，只分转和不转，目标 40 rad/s。
 * 未使能时输出关掉。摩擦轮没开时不拨弹。 */
#include "osal/period.h"
#include "cdd/operate/operate.h"
#include "cdd/motor/motor_init.h"

#define SHOOT_FRICTION_CMD    6000
#define SHOOT_TRIGGER_RAD_S   40.0f

static bool trigger_active;

static void shoot_ctrl_run(void *argument)
{
    bool enabled = false;
    bool friction = false;
    bool shoot = false;
    int16_t friction_cmd;

    (void)argument;
    (void)operate_get(OPERATE_ID_ENABLE, &enabled, sizeof enabled);
    (void)operate_get(OPERATE_ID_FRICTION, &friction, sizeof friction);
    (void)operate_get(OPERATE_ID_SHOOT, &shoot, sizeof shoot);
    friction_cmd = (enabled && friction) ? SHOOT_FRICTION_CMD : 0;
    dji_motor_set_cmd(&shoot_fric_0, friction_cmd);
    dji_motor_set_cmd(&shoot_fric_1, friction_cmd);
    dji_motor_set_cmd(&shoot_fric_2, friction_cmd);
    if (enabled)
    {
        if (!trigger_active)
        {
            motor_set_active(&shoot_m2006.base, true);
            trigger_active = true;
        }
        (void)motor_set_target(&shoot_m2006.base, (friction && shoot) ? SHOOT_TRIGGER_RAD_S : 0.0f);
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
