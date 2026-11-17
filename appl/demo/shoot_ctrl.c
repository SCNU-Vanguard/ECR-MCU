/* 发射：CAN3。三个摩擦轮直接给原始输出，全正向，开时固定 SHOOT_FRICTION_CMD。
 * 拨弹 m2006 速度环，只分转和不转，目标 SHOOT_TRIGGER_RAD_S。
 * 未使能时输出关掉。摩擦轮没开时不拨弹。 */
#include "osal/period.h"
#include "cdd/operate/operate.h"
#include "cdd/motor/motor_init.h"
#include "component/common/types.h"

#define SHOOT_FRICTION_CMD    6000
#define SHOOT_TRIGGER_RAD_S   40.0f

static dji_motor_t *const shoot_friction[] =
{
    &shoot_fric_0,
    &shoot_fric_1,
    &shoot_fric_2,
};

static bool trigger_active;

static void shoot_ctrl_run(void *argument)
{
    operate_snap_t op = {0};
    bool enabled;
    bool friction;
    bool shoot;
    int16_t friction_cmd;

    (void)argument;
    (void)operate_get_snapshot(&op);
    enabled = op.enabled;
    friction = op.friction;
    shoot = op.shoot;
    friction_cmd = (enabled && friction) ? SHOOT_FRICTION_CMD : 0;
    for (uint32_t index = 0; index < ARRAY_SIZE(shoot_friction); index++)
    {
        dji_motor_set_cmd(shoot_friction[index], friction_cmd);
    }
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
