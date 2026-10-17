/* demo：CAN3（FDCAN3 / CAN_BUS2）多电机。3508、6020、3519、4310 各一台，ID 与另外两路相同。
 * 3508 和 3519 是速度环，6020 和 4310 是角度环串速度环。1000 Hz：整条总线计算后一次发出。
 * shell：gimbal_speed / gimbal_angle 设目标，gimbal_on / gimbal_off 开关（可带电机名，不带则四台一起），
 * gimbal_pid 调一台的 PID，gimbal_info 看状态。DM 的 on 会再发一次使能帧。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "robot/demo/robot_cfg.h"
#include "appl/demo/bus_shell.h"

static const demo_bus_motor_t gimbal_motors[] =
{
    { "3508", &gimbal_m3508.base,   NULL },
    { "6020", &gimbal_gm6020.base,  NULL },
    { "3519", &gimbal_dm3519.base,  &gimbal_dm3519 },
    { "4310", &gimbal_dm4310.base,  &gimbal_dm4310 },
};

static int32_t gimbal_ctrl_init(void)
{
    return demo_bus_start(gimbal_motors, 4);
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

#ifdef CONFIG_COMPONENT_SHELL
static int shell_gimbal_speed(int argc, char *argv[])
{
    return demo_bus_shell_target("gimbal_speed", gimbal_motors, 4, MOTOR_CONTROL_SPEED, argc, argv);
}
SHELL_CMD(gimbal_speed, shell_gimbal_speed, set_3508_or_3519_speed_rad_s)

static int shell_gimbal_angle(int argc, char *argv[])
{
    return demo_bus_shell_target("gimbal_angle", gimbal_motors, 4, MOTOR_CONTROL_ANGLE, argc, argv);
}
SHELL_CMD(gimbal_angle, shell_gimbal_angle, set_6020_or_4310_angle_rad)

static int shell_gimbal_on(int argc, char *argv[])
{
    return demo_bus_shell_active("gimbal_on", gimbal_motors, 4, true, argc, argv);
}
SHELL_CMD(gimbal_on, shell_gimbal_on, enable_gimbal_bus_motors)

static int shell_gimbal_off(int argc, char *argv[])
{
    return demo_bus_shell_active("gimbal_off", gimbal_motors, 4, false, argc, argv);
}
SHELL_CMD(gimbal_off, shell_gimbal_off, disable_gimbal_bus_motors)

static int shell_gimbal_pid(int argc, char *argv[])
{
    return demo_bus_shell_pid("gimbal_pid", gimbal_motors, 4, argc, argv);
}
SHELL_CMD(gimbal_pid, shell_gimbal_pid, show_or_set_one_gimbal_motor_pid)

static int shell_gimbal_info(int argc, char *argv[])
{
    return demo_bus_shell_info("gimbal_info", gimbal_motors, 4, argc, argv);
}
SHELL_CMD(gimbal_info, shell_gimbal_info, show_gimbal_bus_motor_state)
#endif
