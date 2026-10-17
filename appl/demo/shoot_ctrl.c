/* demo：CAN1（FDCAN1 / CAN_BUS0）多电机。3508、6020、3519、4310 各一台。
 * 3508 和 3519 是速度环，6020 和 4310 是角度环串速度环。1000 Hz：整条总线计算后一次发出。
 * shell：shoot_speed / shoot_angle 设目标，shoot_on / shoot_off 开关（可带电机名，不带则四台一起），
 * shoot_pid 调一台的 PID，shoot_info 看状态。DM 的 on 会再发一次使能帧。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "robot/demo/robot_cfg.h"
#include "appl/demo/bus_shell.h"

static const demo_bus_motor_t shoot_motors[] =
{
    { "3508", &shoot_m3508.base,   NULL },
    { "6020", &shoot_gm6020.base,  NULL },
    { "3519", &shoot_dm3519.base,  &shoot_dm3519 },
    { "4310", &shoot_dm4310.base,  &shoot_dm4310 },
};

static int32_t shoot_ctrl_init(void)
{
    return demo_bus_start(shoot_motors, 4);
}
INIT_APP_STAGE2(shoot_ctrl_init);

static void shoot_ctrl_run(void *argument)
{
    (void)argument;
    (void)motor_compute_bus(CAN_BUS0);
    (void)motor_send_bus(CAN_BUS0);
}
/* 任务名 shoot_ctrl_task，1000 Hz，中优先级，中栈 */
PERIOD_TASK_DEFINE(shoot_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, shoot_ctrl_run, NULL);

#ifdef CONFIG_COMPONENT_SHELL
static int shell_shoot_speed(int argc, char *argv[])
{
    return demo_bus_shell_target("shoot_speed", shoot_motors, 4, MOTOR_CONTROL_SPEED, argc, argv);
}
SHELL_CMD(shoot_speed, shell_shoot_speed, set_3508_or_3519_speed_rad_s)

static int shell_shoot_angle(int argc, char *argv[])
{
    return demo_bus_shell_target("shoot_angle", shoot_motors, 4, MOTOR_CONTROL_ANGLE, argc, argv);
}
SHELL_CMD(shoot_angle, shell_shoot_angle, set_6020_or_4310_angle_rad)

static int shell_shoot_on(int argc, char *argv[])
{
    return demo_bus_shell_active("shoot_on", shoot_motors, 4, true, argc, argv);
}
SHELL_CMD(shoot_on, shell_shoot_on, enable_shoot_bus_motors)

static int shell_shoot_off(int argc, char *argv[])
{
    return demo_bus_shell_active("shoot_off", shoot_motors, 4, false, argc, argv);
}
SHELL_CMD(shoot_off, shell_shoot_off, disable_shoot_bus_motors)

static int shell_shoot_pid(int argc, char *argv[])
{
    return demo_bus_shell_pid("shoot_pid", shoot_motors, 4, argc, argv);
}
SHELL_CMD(shoot_pid, shell_shoot_pid, show_or_set_one_shoot_motor_pid)

static int shell_shoot_info(int argc, char *argv[])
{
    return demo_bus_shell_info("shoot_info", shoot_motors, 4, argc, argv);
}
SHELL_CMD(shoot_info, shell_shoot_info, show_shoot_bus_motor_state)
#endif
