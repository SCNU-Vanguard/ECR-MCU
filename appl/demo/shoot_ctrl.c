/* 发射：CAN1。左拨杆（switch_left，上=1）开关 m2006，速度恒定。
 * 右拨杆（switch_right，上=1）开关三个摩擦轮。旧步兵摩擦轮把目标转速原值直接发给电调，不经过速度环。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/remote/dt7/dt7.h"
#include "motor_config.h"

#define SHOOT_SWITCH_ON            1u
#define SHOOT_TRIGGER_RAD_S        40.0f
#define SHOOT_FRICTION_CMD         8000

static int32_t shoot_ctrl_init(void)
{
    motor_set_active(&shoot_m2006.base, true);
    return ERR_OK;
}
INIT_APP_STAGE2(shoot_ctrl_init);

static void shoot_ctrl_run(void *argument)
{
    dt7_data_t remote;
    bool trigger_on;
    bool friction_on;
    int16_t friction_cmd;

    (void)argument;
    dt7_get(&remote);
    trigger_on = remote.switch_left == SHOOT_SWITCH_ON;
    friction_on = remote.switch_right == SHOOT_SWITCH_ON;
    (void)motor_set_target(&shoot_m2006.base, trigger_on ? SHOOT_TRIGGER_RAD_S : 0.0f);
    friction_cmd = friction_on ? SHOOT_FRICTION_CMD : 0;
    dji_motor_set_cmd(&shoot_fric_0, friction_cmd);
    dji_motor_set_cmd(&shoot_fric_1, friction_cmd);
    dji_motor_set_cmd(&shoot_fric_2, (int16_t)(-friction_cmd));
    (void)motor_compute_bus(CAN_BUS0);
    (void)motor_send_bus(CAN_BUS0);
}
PERIOD_TASK_DEFINE(shoot_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, shoot_ctrl_run, NULL);
