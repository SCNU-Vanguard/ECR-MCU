/* demo：GM6020 位置速度环验证（chassis_motor，角度环 -> 速度环）。
 * 初始化（INIT_APP_STAGE2）：设为角度模式并打开输出；电机第一次有反馈时，驱动会把目标自动设为当前角度（不会猛转到 0）。
 * 1000 Hz 任务：每个周期计算 + 发送自己所在总线的控制帧。
 * shell：chassis_angle 设目标角度，chassis_on / chassis_off 开关输出，chassis_pid 调参，chassis_info 看状态。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/motor/motor.h"
#include "cdd/motor/motor_shell.h"
#include "robot/demo/robot_cfg.h"

static int32_t chassis_ctrl_init(void)
{
    (void)motor_set_target(&chassis_motor.base, MOTOR_CONTROL_ANGLE, 0.0f);
    motor_set_active(&chassis_motor.base, true);            /* 目标在第一次有反馈时自动设为当前状态 */
    return ERR_OK;
}
INIT_APP_STAGE2(chassis_ctrl_init);

static void chassis_ctrl_run(void *argument)
{
    motor_t *motor = &chassis_motor.base;

    (void)argument;
    (void)motor_compute(motor);
    (void)motor_send_bus(motor->bus);
}
/* 任务名 chassis_ctrl_task，1000 Hz，中优先级，中栈 */
PERIOD_TASK_DEFINE(chassis_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, chassis_ctrl_run, NULL);

#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"

/* chassis_angle <rad>：设目标角度（单圈，自动回绕到 ±pi） */
static int shell_chassis_angle(int argc, char *argv[])
{
    float angle;

    if (argc != 2 || !motor_shell_parse_float(argv[1], &angle))
    {
        shell_out("usage: chassis_angle [rad]\r\n");
        return -1;
    }
    return motor_set_target(&chassis_motor.base, MOTOR_CONTROL_ANGLE, angle);
}
SHELL_CMD(chassis_angle, shell_chassis_angle, set_chassis_motor_target_angle_rad)

/* chassis_on：打开输出，目标角度自动设为当前角度；chassis_off：关闭输出（发 0） */
static int shell_chassis_on(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    motor_set_active(&chassis_motor.base, true);
    return 0;
}
SHELL_CMD(chassis_on, shell_chassis_on, enable_chassis_motor_output_holding_current_angle)

static int shell_chassis_off(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    motor_set_active(&chassis_motor.base, false);
    return 0;
}
SHELL_CMD(chassis_off, shell_chassis_off, disable_chassis_motor_output)

static int shell_chassis_pid(int argc, char *argv[])
{
    return motor_shell_pid("chassis", &chassis_motor.base, argc, argv);
}
SHELL_CMD(chassis_pid, shell_chassis_pid, show_or_set_chassis_motor_pid)

static int shell_chassis_info(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    motor_shell_info("chassis", &chassis_motor.base);
    return 0;
}
SHELL_CMD(chassis_info, shell_chassis_info, show_chassis_motor_state)
#endif
