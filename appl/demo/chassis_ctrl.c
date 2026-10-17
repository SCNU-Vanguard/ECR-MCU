/* demo：CAN2（FDCAN2 / CAN_BUS1）多电机。3508、6020、3519、4310 各一台，ID 与另外两路相同。
 * 3508 和 3519 是速度环，6020 和 4310 是角度环串速度环。1000 Hz：整条总线计算后一次发出。
 * shell：chassis_speed / chassis_angle 设目标，chassis_on / chassis_off 开关（可带电机名，不带则四台一起），
 * chassis_pid 调一台的 PID，chassis_info 看状态。DM 的 on 会再发一次使能帧。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "robot/demo/robot_cfg.h"
#include "appl/demo/bus_shell.h"

static const demo_bus_motor_t chassis_motors[] =
{
    { "3508", &chassis_m3508.base,   NULL },
    { "6020", &chassis_gm6020.base,  NULL },
    { "3519", &chassis_dm3519.base,  &chassis_dm3519 },
    { "4310", &chassis_dm4310.base,  &chassis_dm4310 },
};

static int32_t chassis_ctrl_init(void)
{
    return demo_bus_start(chassis_motors, 4);
}
INIT_APP_STAGE2(chassis_ctrl_init);

static void chassis_ctrl_run(void *argument)
{
    (void)argument;
    (void)motor_compute_bus(CAN_BUS1);
    (void)motor_send_bus(CAN_BUS1);
}
/* 任务名 chassis_ctrl_task，1000 Hz，中优先级，中栈 */
PERIOD_TASK_DEFINE(chassis_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, chassis_ctrl_run, NULL);

#ifdef CONFIG_COMPONENT_SHELL
static int shell_chassis_speed(int argc, char *argv[])
{
    return demo_bus_shell_target("chassis_speed", chassis_motors, 4, MOTOR_CONTROL_SPEED, argc, argv);
}
SHELL_CMD(chassis_speed, shell_chassis_speed, set_3508_or_3519_speed_rad_s)

static int shell_chassis_angle(int argc, char *argv[])
{
    return demo_bus_shell_target("chassis_angle", chassis_motors, 4, MOTOR_CONTROL_ANGLE, argc, argv);
}
SHELL_CMD(chassis_angle, shell_chassis_angle, set_6020_or_4310_angle_rad)

static int shell_chassis_on(int argc, char *argv[])
{
    return demo_bus_shell_active("chassis_on", chassis_motors, 4, true, argc, argv);
}
SHELL_CMD(chassis_on, shell_chassis_on, enable_chassis_bus_motors)

static int shell_chassis_off(int argc, char *argv[])
{
    return demo_bus_shell_active("chassis_off", chassis_motors, 4, false, argc, argv);
}
SHELL_CMD(chassis_off, shell_chassis_off, disable_chassis_bus_motors)

static int shell_chassis_pid(int argc, char *argv[])
{
    return demo_bus_shell_pid("chassis_pid", chassis_motors, 4, argc, argv);
}
SHELL_CMD(chassis_pid, shell_chassis_pid, show_or_set_one_chassis_motor_pid)

static int shell_chassis_info(int argc, char *argv[])
{
    return demo_bus_shell_info("chassis_info", chassis_motors, 4, argc, argv);
}
SHELL_CMD(chassis_info, shell_chassis_info, show_chassis_bus_motor_state)
#endif
