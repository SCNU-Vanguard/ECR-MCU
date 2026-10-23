/* 底盘：CAN2 四个 m3508，麦克纳姆，另有一台 MF9025。左摇杆平移，拨轮只开关恒定自旋。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/remote/dt7/dt7.h"
#include "cdd/motor/motor_init.h"
#include "chassis_config.h"

#define CHASSIS_STICK_MAX       660.0f
#define CHASSIS_STICK_IGNORE    20
#define CHASSIS_DIAL_IGNORE     100
#define CHASSIS_SPEED_MAX_M_S   1.5f
#define CHASSIS_SPIN_RAD_S      2.0f

static dji_motor_t *const chassis_wheel[CHASSIS_WHEEL_COUNT] =
{
    &chassis_m3508_0,
    &chassis_m3508_1,
    &chassis_m3508_2,
    &chassis_m3508_3,
};

static float stick_to_speed(int16_t raw)
{
    if (raw > -CHASSIS_STICK_IGNORE && raw < CHASSIS_STICK_IGNORE)
    {
        return 0.0f;
    }
    return (float)raw / CHASSIS_STICK_MAX * CHASSIS_SPEED_MAX_M_S;
}

static int32_t chassis_ctrl_init(void)
{
    for (uint32_t wheel_index = 0; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
    {
        motor_set_active(&chassis_wheel[wheel_index]->base, true);
    }
    (void)lk_motor_run(&mf9025);
    motor_set_active(&mf9025.base, true);
    return ERR_OK;
}
INIT_APP_STAGE2(chassis_ctrl_init);

static void chassis_ctrl_run(void *argument)
{
    dt7_data_t remote;
    chassis_speed_t speed;
    float wheel_rad_s[CHASSIS_WHEEL_COUNT];

    (void)argument;
    dt7_get(&remote);
    speed.forward_m_s = stick_to_speed(remote.channel[3]);
    speed.right_m_s = stick_to_speed(remote.channel[2]);
    if (remote.wheel > CHASSIS_DIAL_IGNORE)
    {
        speed.spin_rad_s = CHASSIS_SPIN_RAD_S;
    }
    else if (remote.wheel < -CHASSIS_DIAL_IGNORE)
    {
        speed.spin_rad_s = -CHASSIS_SPIN_RAD_S;
    }
    else
    {
        speed.spin_rad_s = 0.0f;
    }
    if (mecanum_solve(&mecanum, &speed, wheel_rad_s) == ERR_OK)
    {
        for (uint32_t wheel_index = 0; wheel_index < CHASSIS_WHEEL_COUNT; wheel_index++)
        {
            (void)motor_set_target(&chassis_wheel[wheel_index]->base, wheel_rad_s[wheel_index] * M3508_GEAR_RATIO);
        }
    }
    (void)motor_compute_bus(CAN_BUS1);
    (void)motor_send_bus(CAN_BUS1);
}
PERIOD_TASK_DEFINE(chassis_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, chassis_ctrl_run, NULL);
