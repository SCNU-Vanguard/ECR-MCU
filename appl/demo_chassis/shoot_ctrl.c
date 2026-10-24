/* 拨弹：CAN3，m2006，速度环。云台只给转或不转，目标速度恒定。
 * 整机失能或板间离线时输出关掉。允许动作但不开火时，速度目标为 0。 */
#include "osal/period.h"
#include "cdd/motor/motor_init.h"
#include "cdd/interboard/interboard.h"
#include "interboard_config.h"

#define SHOOT_TRIGGER_RAD_S  40.0f

static bool trigger_active;

static void shoot_ctrl_run(void *argument)
{
    interboard_link_t link;
    interboard_request_t request;
    bool allowed;
    bool spinning;

    (void)argument;
    interboard_get_link(&link);
    allowed = link.online && interboard_load_request(&request) && request.motion.enabled != 0u;
    spinning = allowed && request.shoot.trigger_on != 0u;
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
