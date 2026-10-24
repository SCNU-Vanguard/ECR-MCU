/* 三个摩擦轮，CAN2，直接给原始输出。全正向，开时固定 6000，关时 0。 */
#include "osal/period.h"
#include "cdd/remote/dt7/dt7.h"
#include "cdd/motor/motor_init.h"

#define DT7_SWITCH_UP       1u
#define DT7_SWITCH_MID      3u
#define SHOOT_FRICTION_CMD  6000

static void shoot_ctrl_run(void *argument)
{
    dt7_data_t remote;
    bool friction_on;
    int16_t friction_cmd;

    (void)argument;
    dt7_get(&remote);
    friction_on = remote.online
        && (remote.switch_left == DT7_SWITCH_UP || remote.switch_left == DT7_SWITCH_MID)
        && (remote.switch_right == DT7_SWITCH_UP || remote.switch_right == DT7_SWITCH_MID);
    friction_cmd = friction_on ? SHOOT_FRICTION_CMD : 0;
    dji_motor_set_cmd(&shoot_fric_0, friction_cmd);
    dji_motor_set_cmd(&shoot_fric_1, friction_cmd);
    dji_motor_set_cmd(&shoot_fric_2, friction_cmd);
    (void)motor_send_bus(CAN_BUS2);
}
PERIOD_TASK_DEFINE(shoot_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, shoot_ctrl_run, NULL);
