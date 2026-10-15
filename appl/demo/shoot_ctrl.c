/* demo：M3508 速度环验证（shoot_motor）。
 * 初始化（INIT_APP_STAGE2）：设为速度模式并打开输出；电机第一次有反馈时，驱动会把目标自动设为 0。
 * 1000 Hz 任务：每个周期计算 + 发送自己所在总线的控制帧。
 * shell：shoot_speed 设目标转速，shoot_on / shoot_off 开关输出，shoot_pid 调参，shoot_info 看状态。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/motor/motor.h"
#include "cdd/motor/motor_shell.h"
#include "robot/demo/robot_cfg.h"

static int32_t shoot_ctrl_init(void)
{
    (void)motor_set_target(&shoot_motor.base, MOTOR_CONTROL_SPEED, 0.0f);
    motor_set_active(&shoot_motor.base, true);              /* 目标在第一次有反馈时自动设为当前状态 */
    return ERR_OK;
}
INIT_APP_STAGE2(shoot_ctrl_init);

static void shoot_ctrl_run(void *argument)
{
    motor_t *motor = &shoot_motor.base;

    (void)argument;
    (void)motor_compute(motor);
    (void)motor_send_bus(motor->bus);
}
/* 任务名 shoot_ctrl_task，1000 Hz，中优先级，中栈 */
PERIOD_TASK_DEFINE(shoot_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, shoot_ctrl_run, NULL);

#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"

/* shoot_speed <rad/s>：设目标转速 */
static int shell_shoot_speed(int argc, char *argv[])
{
    float speed;

    if (argc != 2 || !motor_shell_parse_float(argv[1], &speed))
    {
        shell_out("usage: shoot_speed [rad/s]\r\n");
        return -1;
    }
    return motor_set_target(&shoot_motor.base, MOTOR_CONTROL_SPEED, speed);
}
SHELL_CMD(shoot_speed, shell_shoot_speed, set_shoot_motor_target_speed_rad_s)

/* shoot_on：打开输出，目标速度自动回到 0；shoot_off：关闭输出（发 0） */
static int shell_shoot_on(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    motor_set_active(&shoot_motor.base, true);
    return 0;
}
SHELL_CMD(shoot_on, shell_shoot_on, enable_shoot_motor_output_with_target_0)

static int shell_shoot_off(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    motor_set_active(&shoot_motor.base, false);
    return 0;
}
SHELL_CMD(shoot_off, shell_shoot_off, disable_shoot_motor_output)

static int shell_shoot_pid(int argc, char *argv[])
{
    return motor_shell_pid("shoot", &shoot_motor.base, argc, argv);
}
SHELL_CMD(shoot_pid, shell_shoot_pid, show_or_set_shoot_motor_pid)

static int shell_shoot_info(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    motor_shell_info("shoot", &shoot_motor.base);
    return 0;
}
SHELL_CMD(shoot_info, shell_shoot_info, show_shoot_motor_state)
#endif
