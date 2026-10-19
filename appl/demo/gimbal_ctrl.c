/* demo：CAN3（FDCAN3 / CAN_BUS2）多电机。3508、6020、3519、4310 各一台，ID 与另外两路相同。
 * 3508 和 3519 是速度环，6020 和 4310 是角度环串速度环。1000 Hz：整条总线计算后一次发出。
 * 初始化打开四台输出，并给两台达妙各发一次使能帧。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "robot/demo/robot_config.h"

static int32_t gimbal_ctrl_init(void)
{
    int32_t first_error;

    motor_set_active(&gimbal_m3508.base, true);
    motor_set_active(&gimbal_gm6020.base, true);
    motor_set_active(&gimbal_s3519.base, true);
    motor_set_active(&gimbal_j4310.base, true);
    first_error = dm_motor_enable(&gimbal_s3519);
    if (first_error != ERR_OK)
    {
        return first_error;
    }
    return dm_motor_enable(&gimbal_j4310);
}
INIT_APP_STAGE2(gimbal_ctrl_init);

static void gimbal_ctrl_run(void *argument)
{
    (void)argument;
    (void)motor_compute_bus(CAN_BUS2);
    (void)motor_send_bus(CAN_BUS2);
}
/* 任务名 gimbal_ctrl_task，1000 Hz，中优先级，中栈 */
PERIOD_TASK_DEFINE(gimbal_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, gimbal_ctrl_run, NULL);
