/* 三个摩擦轮，CAN2，直接给原始输出。全正向，开时固定 6000，关时 0。
 * 未使能时不开。 */
#include "osal/period.h"
#include "cdd/operate/operate.h"
#include "cdd/motor/motor_init.h"

#define SHOOT_FRICTION_CMD  6000

static void shoot_ctrl_run(void *argument)
{
    bool enabled = false;
    bool friction = false;
    int16_t friction_cmd;

    (void)argument;
    (void)operate_get(OPERATE_ID_ENABLE, &enabled, sizeof enabled);
    (void)operate_get(OPERATE_ID_FRICTION, &friction, sizeof friction);
    friction_cmd = (enabled && friction) ? SHOOT_FRICTION_CMD : 0;
    dji_motor_set_cmd(&shoot_fric_0, friction_cmd);
    dji_motor_set_cmd(&shoot_fric_1, friction_cmd);
    dji_motor_set_cmd(&shoot_fric_2, friction_cmd);
    (void)motor_send_bus(CAN_BUS2);
}
PERIOD_TASK_DEFINE(shoot_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, shoot_ctrl_run, NULL);
