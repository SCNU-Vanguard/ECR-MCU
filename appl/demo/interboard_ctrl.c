/* 板间主机：500 Hz。每次只发一种命令，三种轮流，每种大约 166 Hz。
 * control 的平移和自旋与底盘任务同一套摇杆换算，yaw 用 INS。motion、referee 没有请求载荷。 */
#include "osal/period.h"
#include "cdd/remote/dt7/dt7.h"
#include "cdd/imu/ins/ins.h"
#include "cdd/interboard/interboard.h"
#include "interboard_config.h"

#define INTERBOARD_STICK_MAX      660.0f
#define INTERBOARD_STICK_IGNORE   20
#define INTERBOARD_DIAL_IGNORE    100
#define INTERBOARD_SPEED_MAX_M_S  1.5f
#define INTERBOARD_SPIN_RAD_S     2.0f

static float stick_to_speed(int16_t raw)
{
    if (raw > -INTERBOARD_STICK_IGNORE && raw < INTERBOARD_STICK_IGNORE)
    {
        return 0.0f;
    }
    return (float)raw / INTERBOARD_STICK_MAX * INTERBOARD_SPEED_MAX_M_S;
}

static void post_control(void)
{
    dt7_data_t remote;
    ins_attitude_t attitude;
    interboard_control_t control;

    dt7_get(&remote);
    ins_get_attitude(&attitude);
    control.forward_m_s = stick_to_speed(remote.channel[3]);
    control.right_m_s = stick_to_speed(remote.channel[2]);
    if (remote.wheel > INTERBOARD_DIAL_IGNORE)
    {
        control.spin_rad_s = INTERBOARD_SPIN_RAD_S;
    }
    else if (remote.wheel < -INTERBOARD_DIAL_IGNORE)
    {
        control.spin_rad_s = -INTERBOARD_SPIN_RAD_S;
    }
    else
    {
        control.spin_rad_s = 0.0f;
    }
    control.yaw_rad = attitude.valid ? attitude.yaw_rad : 0.0f;
    (void)interboard_post(INTERBOARD_CMD_CONTROL, &control, sizeof control);
}

static void interboard_ctrl_run(void *argument)
{
    static uint8_t which;

    (void)argument;
    if (which == 0u)
    {
        post_control();
    }
    else if (which == 1u)
    {
        (void)interboard_post(INTERBOARD_CMD_MOTION, NULL, 0u);
    }
    else
    {
        (void)interboard_post(INTERBOARD_CMD_REFEREE, NULL, 0u);
    }
    which = (uint8_t)((which + 1u) % 3u);
}
PERIOD_TASK_DEFINE(interboard_ctrl_task, OS_FREQ_500HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, interboard_ctrl_run, NULL);
