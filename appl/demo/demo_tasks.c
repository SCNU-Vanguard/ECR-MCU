#include "robot/demo/robot_cfg.h"
#include "osal/period.h"
#include "component/log/log.h"
#ifdef CONFIG_CDD_REMOTE_DT7
#include "cdd/remote/dt7.h"
static dt7_data_t rc;
#endif
#ifdef CONFIG_CDD_MOTOR_DJI
#include "cdd/motor/dji_motor/dji_motor.h"
extern dji_motor_t chassis_motor[4];                 /* robot/demo/robot_init.c */
#endif
#ifdef CONFIG_CDD_IMU_BMI088
#include "cdd/imu/bmi088/bmi088.h"
static bmi088_raw_t imu;
#endif
#ifdef CONFIG_BSP_CAN
#include "bsp/can/bsp_can.h"
#endif

/* 状态灯/蜂鸣器：CONFIG_SELFTEST = y 时由 selftest 下的任务接管，这里不再去动它们 */
#if !defined(CONFIG_SELFTEST)
#ifdef CONFIG_CDD_LED_WS2812
#include "cdd/led/ws2812/ws2812.h"
#define STATUS_LED 1
#endif
#ifdef CONFIG_CDD_BUZZER
#include "cdd/buzzer/buzzer.h"
#define STATUS_BUZZER 1
#endif
#endif

/* 1 kHz 控制任务：遥控器 -> 目标 -> 电机命令 -> 发 CAN。
 * 这里只放骨架：控制律还没写，所以在线时命令也是 0（占位）；真正的控制律写在标 TODO 的地方。
 * 失控保护已经是对的：遥控器离线（或没编译遥控器驱动）时命令一定是 0。 */
static void ctrl_task(void *arg)
{
    (void)arg;
#ifdef CONFIG_CDD_REMOTE_DT7
    dt7_get(&rc);
    bool rc_ok = rc.online;
#else
    bool rc_ok = false;
#endif
#ifdef CONFIG_CDD_MOTOR_DJI
    for (int i = 0; i < 4; i++) {
        int16_t cmd = 0;                                    /* 离线 -> 0 */
        if (rc_ok) {
            cmd = 0;                                        /* TODO：由 rc.ch[] 算出第 i 个电机的命令 */
        }
        dji_motor_set_cmd(&chassis_motor[i], cmd);
    }
    (void)dji_motor_send_all();
#else
    (void)rc_ok;
#endif
}
PERIOD_TASK_DEFINE(demo_ctrl, 1000, 1, 256, ctrl_task, NULL);

/* 500 Hz：读 IMU 原始数据（姿态解算在 component/filter 里接） */
static void imu_task(void *arg)
{
    (void)arg;
#ifdef CONFIG_CDD_IMU_BMI088
    (void)bmi088_read_raw(&imu);
#endif
}
PERIOD_TASK_DEFINE(demo_imu, 500, 2, 256, imu_task, NULL);

/* 10 Hz：状态灯 + 每秒一行状态日志 */
static void status_task(void *arg)
{
    static uint32_t n;
    (void)arg;
#ifdef CONFIG_CDD_REMOTE_DT7
    dt7_get(&rc);
#endif
#ifdef STATUS_BUZZER
    if (n == 0u) { buzzer_on(2000); }                        /* 开机"滴"一声 */
    if (n == 3u) { buzzer_off(); }
#endif
#ifdef STATUS_LED
#ifdef CONFIG_CDD_REMOTE_DT7
    if (rc.online) { ws2812_set(0, 0, 32, 0); } else { ws2812_set(0, 32, 0, 0); }   /* 绿=遥控器在线 红=离线 */
#else
    ws2812_set(0, 0, 0, 32);
#endif
    (void)ws2812_refresh();
#endif
    if (++n % 10u == 0u) {
#ifdef CONFIG_CDD_REMOTE_DT7
        LOGI("stat", "rc=%d s=%u/%u ch=%d,%d", rc.online, rc.s1, rc.s2, rc.ch[0], rc.ch[1]);
#endif
#ifdef CONFIG_CDD_IMU_BMI088
        LOGI("stat", "gyro=%d,%d,%d", imu.gyro[0], imu.gyro[1], imu.gyro[2]);
#endif
#ifdef CONFIG_BSP_CAN
        const bsp_can_stats_t *c0 = bsp_can_stats(CAN_BUS0);
        LOGI("stat", "can0 rx=%lu tx=%lu", (unsigned long)c0->rx, (unsigned long)c0->tx);
#endif
    }
}
PERIOD_TASK_DEFINE(demo_status, 10, 10, 512, status_task, NULL);
