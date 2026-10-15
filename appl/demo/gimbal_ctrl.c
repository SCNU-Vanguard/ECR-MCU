/* demo：DM4310 MIT 软件转矩 + 位置速度环验证（gimbal_motor，角度环 -> 速度环 -> 转矩 t_ff，Kp = Kd = 0）。
 * 初始化（INIT_APP_STAGE2，电机已在 STAGE1 绑定好、CAN 已在 BSP 阶段启动）：设为角度模式、打开输出，并发使能帧；
 *   电机第一次有反馈时，驱动会把目标自动设为当前角度（不会猛转到 0）。
 * 1000 Hz 任务：每个周期计算 + 发送这个电机的控制帧。
 * 注意：使能帧只在初始化和 gimbal_on 里各发一次，不会自动重发。电机比 MCU 晚上电、或运行中掉回失能时，要手动 gimbal_on。
 * shell：gimbal_angle 设目标角度，gimbal_on / gimbal_off 使能和失能，gimbal_pid 调参，gimbal_info 看状态。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/motor/motor.h"
#include "cdd/motor/motor_shell.h"
#include "robot/demo/robot_cfg.h"

static int32_t gimbal_ctrl_init(void)
{
    (void)motor_set_target(&gimbal_motor.base, MOTOR_CONTROL_ANGLE, 0.0f);
    motor_set_active(&gimbal_motor.base, true);             /* 目标在第一次有反馈时自动设为当前角度 */
    return dm_motor_enable(&gimbal_motor);
}
INIT_APP_STAGE2(gimbal_ctrl_init);

static void gimbal_ctrl_run(void *argument)
{
    (void)argument;
    dm_motor_compute(&gimbal_motor);
    (void)dm_motor_send(&gimbal_motor);
}
/* 任务名 gimbal_ctrl_task，1000 Hz，中优先级，中栈 */
PERIOD_TASK_DEFINE(gimbal_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, gimbal_ctrl_run, NULL);

#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"

/* gimbal_angle <rad>：设目标角度（单圈，自动回绕到 ±pi） */
static int shell_gimbal_angle(int argc, char *argv[])
{
    float angle;

    if (argc != 2 || !motor_shell_parse_float(argv[1], &angle))
    {
        shell_out("usage: gimbal_angle [rad]\r\n");
        return -1;
    }
    return motor_set_target(&gimbal_motor.base, MOTOR_CONTROL_ANGLE, angle);
}
SHELL_CMD(gimbal_angle, shell_gimbal_angle, set_gimbal_motor_target_angle_rad)

/* gimbal_on：发使能帧并打开输出，目标角度自动设为当前角度；gimbal_off：关闭输出并发失能帧 */
static int shell_gimbal_on(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    motor_set_active(&gimbal_motor.base, true);
    return dm_motor_enable(&gimbal_motor);
}
SHELL_CMD(gimbal_on, shell_gimbal_on, enable_gimbal_motor_holding_current_angle)

static int shell_gimbal_off(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    motor_set_active(&gimbal_motor.base, false);
    return dm_motor_disable(&gimbal_motor);
}
SHELL_CMD(gimbal_off, shell_gimbal_off, disable_gimbal_motor_output_and_motor)

static int shell_gimbal_pid(int argc, char *argv[])
{
    return motor_shell_pid("gimbal", &gimbal_motor.base, argc, argv);
}
SHELL_CMD(gimbal_pid, shell_gimbal_pid, show_or_set_gimbal_motor_pid)

static int shell_gimbal_info(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    motor_shell_info("gimbal", &gimbal_motor.base);
    shell_out("  dm_state = %u (1 = enabled)\r\n", (unsigned)gimbal_motor.state);
    return 0;
}
SHELL_CMD(gimbal_info, shell_gimbal_info, show_gimbal_motor_state)
#endif
