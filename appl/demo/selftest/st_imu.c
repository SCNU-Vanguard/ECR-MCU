/* 自测：BMI088。100 Hz 读原始数据，每秒打印一行 1 秒平均值和判定。
 * 判定依据（量程取上电默认值，以数据手册为准）：
 *   加速度计 ±6 g：1 g 约 5461 LSB，静止时三轴合成应落在 4400..6500
 *   陀螺仪 ±2000 dps：16.384 LSB/dps，静止时每轴平均值应很小（< 150 LSB，约 9 dps） */
#include <math.h>
#include <stdlib.h>
#include "osal/period.h"
#include "cdd/imu/bmi088/bmi088.h"
#include "component/log/log.h"

static void imu_test_task(void *arg)
{
    static int32_t sa[3], sg[3];
    static uint32_t n, fail;
    bmi088_raw_t r;
    (void)arg;

    if (!bmi088_present()) {
        if (++n % 100u == 1u) { LOGW("st_imu", "bmi088 not present (see init log)"); }
        return;
    }
    if (bmi088_read_raw(&r) != 0) { fail++; return; }
    for (int i = 0; i < 3; i++) { sa[i] += r.acc[i]; sg[i] += r.gyro[i]; }

    if (++n % 100u == 0u) {
        int32_t a[3], g[3];
        for (int i = 0; i < 3; i++) { a[i] = sa[i] / 100; g[i] = sg[i] / 100; sa[i] = 0; sg[i] = 0; }
        int norm = (int)sqrtf((float)a[0] * a[0] + (float)a[1] * a[1] + (float)a[2] * a[2]);
        bool acc_ok  = norm > 4400 && norm < 6500;
        bool gyro_ok = abs(g[0]) < 150 && abs(g[1]) < 150 && abs(g[2]) < 150;
        LOGI("st_imu", "acc=%ld,%ld,%ld |a|=%d %s  gyro=%ld,%ld,%ld %s  readfail=%lu",
             (long)a[0], (long)a[1], (long)a[2], norm, acc_ok ? "OK" : "CHECK",
             (long)g[0], (long)g[1], (long)g[2], gyro_ok ? "OK(still)" : "moving/CHECK", (unsigned long)fail);
    }
}
PERIOD_TASK_DEFINE(st_imu, 100, 11, 512, imu_test_task, NULL);
