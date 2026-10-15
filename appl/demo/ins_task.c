/* demo：惯导（姿态解算）任务。
 * 数据流：BMI088 DRDY 中断 -> SPI DMA 读取（全在中断里，见 cdd/imu/bmi088）-> 帧回调里 event_post_isr -> 本任务
 *   本任务：换算物理量（含标定）-> 用 DRDY 时间戳算真实 dt -> ahrs_update（算法由本文件的 INS_ALGORITHM 在编译期决定）-> 发布姿态。
 * 超时：100 ms 没有新帧（DRDY 线卡死、DMA 异常），调用 bmi088_recover 重新拉起采集。
 * shell：ins_info 看姿态和统计，ins_imu 看最近一帧物理量。 */
#include <string.h>
#include "osal/event.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/log/log.h"
#include "component/filter/ahrs.h"
#include "cdd/imu/bmi088/bmi088.h"
#include "appl/demo/ins_task.h"

/* 姿态解算算法（ahrs_algorithm_e）：AHRS_ALGORITHM_VQF / MEKF / MADGWICK / MAHONY / EKF。换算法就改这里，重新编译烧录 */
#define INS_ALGORITHM               AHRS_ALGORITHM_VQF

#define INS_TIMEOUT_MS              100u
#define INS_QUEUE_DEPTH             16u                     /* 1 kHz 下约 16 ms 的缓冲 */
/* 加速度计样本比陀螺仪样本旧过这么多就认为过期，这一拍加速度计不参与校正（只用陀螺积分）：两个加速度计周期 + 1 ms */
#define INS_ACCEL_MAX_AGE_US        (2u * BMI088_ACCEL_SAMPLE_PERIOD_US + 1000u)

typedef struct
{
    uint32_t frame_count;
    uint32_t gap_count;             /* 陀螺仪序号不连续（丢样本） */
    uint32_t stale_accel_count;
    uint32_t reject_count;          /* ahrs_update 拒绝或失败 */
    uint32_t timeout_count;
    float    last_dt_s;
} ins_stats_t;

static ahrs_t ahrs;
static ins_stats_t ins_stats;
static bmi088_sample_t last_sample;
static ins_attitude_t published_attitude;
static volatile uint32_t publish_version;                  /* 奇数 = 正在写，用来让读者拿到一致的快照 */
static bool have_previous_frame;
static uint32_t previous_gyro_timestamp_us;
static uint32_t previous_gyro_sequence;

static void publish_attitude(const bmi088_sample_t *sample)
{
    publish_version++;
    __DMB();
    published_attitude.valid = ahrs.update_count > 0u;
    memcpy(published_attitude.quaternion, ahrs.quaternion, sizeof published_attitude.quaternion);
    published_attitude.roll_rad = ahrs.roll_rad;
    published_attitude.pitch_rad = ahrs.pitch_rad;
    published_attitude.yaw_rad = ahrs.yaw_rad;
    published_attitude.yaw_total_rad = ahrs.yaw_total_rad;
    memcpy(published_attitude.gyro_rad_s, sample->gyro_rad_s, sizeof published_attitude.gyro_rad_s);
    published_attitude.temperature_c = sample->temperature_c;
    published_attitude.update_count = ahrs.update_count;
    __DMB();
    publish_version++;
}

void ins_get_attitude(ins_attitude_t *attitude)
{
    uint32_t version_before;
    uint32_t version_after;

    do
    {
        version_before = publish_version;
        __DMB();
        *attitude = published_attitude;
        __DMB();
        version_after = publish_version;
    } while ((version_before & 1u) != 0u || version_before != version_after);
}

static void ins_run(void *argument, const uint8_t *data, uint16_t length)
{
    bmi088_frame_t frame;
    ahrs_sample_t ahrs_sample;

    (void)argument;
    if (data == NULL)
    {
        ins_stats.timeout_count++;
        have_previous_frame = false;
        ahrs_break_continuity(&ahrs);
        (void)bmi088_recover();
        return;
    }
    if (length != sizeof frame)
    {
        return;
    }
    memcpy(&frame, data, sizeof frame);
    ins_stats.frame_count++;

    float dt_s = 1.0f / (float)BMI088_RATE_HZ;

    if (have_previous_frame)
    {
        dt_s = (float)(uint32_t)(frame.gyro_timestamp_us - previous_gyro_timestamp_us) * 1.0e-6f;
        if ((uint32_t)(frame.gyro_sequence - previous_gyro_sequence) != 1u)
        {
            ins_stats.gap_count++;
            ahrs_break_continuity(&ahrs);                   /* 丢了样本，不能让算法把缺口当连续数据 */
        }
    }
    have_previous_frame = true;
    previous_gyro_timestamp_us = frame.gyro_timestamp_us;
    previous_gyro_sequence = frame.gyro_sequence;
    ins_stats.last_dt_s = dt_s;

    bmi088_convert(&frame, &last_sample);
    memcpy(ahrs_sample.gyro_rad_s, last_sample.gyro_rad_s, sizeof ahrs_sample.gyro_rad_s);
    memcpy(ahrs_sample.accel_m_s2, last_sample.accel_m_s2, sizeof ahrs_sample.accel_m_s2);
    ahrs_sample.dt_s = dt_s;
    ahrs_sample.gyro_clipped = last_sample.gyro_clipped;
    ahrs_sample.accel_clipped = last_sample.accel_clipped;
    ahrs_sample.stationary = false;

    if ((uint32_t)(frame.gyro_timestamp_us - frame.accel_timestamp_us) > INS_ACCEL_MAX_AGE_US)
    {
        ins_stats.stale_accel_count++;
        ahrs_sample.accel_clipped = true;                   /* 借用"加速度计不可信"标志：这一拍只积分陀螺 */
    }

    int32_t update_result = ahrs_update(&ahrs, &ahrs_sample);

    if (update_result == ERR_OK)
    {
        publish_attitude(&last_sample);
    }
    else if (update_result != ERR_BUSY)
    {
        ins_stats.reject_count++;
    }
}
/* 任务名 ins_task：事件驱动，高优先级，大栈（滤波算法的局部矩阵较多） */
EVENT_TASK_DEFINE(ins_task, OS_PRIORITY_HIGH, OS_STACK_LARGE, EVENT_QUEUE, sizeof(bmi088_frame_t), INS_QUEUE_DEPTH, INS_TIMEOUT_MS, ins_run, NULL);

/* 在 DRDY / DMA 完成中断里调用 */
static void ins_on_frame(const bmi088_frame_t *frame, void *context)
{
    (void)context;
    (void)event_post_isr(&ins_task, frame, sizeof *frame);
}

static int32_t ins_init(void)
{
    int32_t ret = ahrs_start(&ahrs, INS_ALGORITHM, 1.0f / (float)BMI088_RATE_HZ);

    if (ret != ERR_OK)
    {
        LOGE("ins", "default algorithm unavailable: %d", (int)ret);
        return ret;
    }
    if (!bmi088_present())
    {
        LOGW("ins", "bmi088 not present, ins disabled");
        return ERR_NODEV;
    }
    bmi088_set_frame_callback(ins_on_frame, NULL);
    return bmi088_start();
}
INIT_APP_STAGE2(ins_init);

#ifdef CONFIG_COMPONENT_SHELL
#include <stdio.h>
#include <stdlib.h>
#include "component/shell/shell_cmd.h"

/* 浮点转成 "-1.234" 形式：当前 C 库没开浮点 printf，自己按整数拆 */
static void format_milli(char *text, size_t size, float value)
{
    long milli = (long)(value * 1000.0f + (value >= 0.0f ? 0.5f : -0.5f));
    const char *sign = (milli < 0) ? "-" : "";

    if (milli < 0)
    {
        milli = -milli;
    }
    snprintf(text, size, "%s%ld.%03ld", sign, milli / 1000, milli % 1000);
}

static void print_milli(const char *label, float value, const char *unit)
{
    char text[20];

    format_milli(text, sizeof text, value);
    shell_out("  %s = %s %s\r\n", label, text, unit);
}

#define RAD_TO_DEG  57.2957795f

static int shell_ins_info(int argc, char *argv[])
{
    ins_attitude_t attitude;
    bmi088_stats_t driver_stats;

    (void)argc;
    (void)argv;
    ins_get_attitude(&attitude);
    bmi088_get_stats(&driver_stats);
    shell_out("ins: algorithm=%s valid=%d updates=%lu\r\n", ahrs_algorithm_name(ahrs.algorithm), (int)attitude.valid,
              (unsigned long)attitude.update_count);
    print_milli("roll", attitude.roll_rad * RAD_TO_DEG, "deg");
    print_milli("pitch", attitude.pitch_rad * RAD_TO_DEG, "deg");
    print_milli("yaw", attitude.yaw_rad * RAD_TO_DEG, "deg");
    print_milli("yaw_total", attitude.yaw_total_rad * RAD_TO_DEG, "deg");
    print_milli("temperature", attitude.temperature_c, "C");
    print_milli("dt", ins_stats.last_dt_s * 1000.0f, "ms");
    shell_out("  frames=%lu gaps=%lu stale_accel=%lu rejects=%lu timeouts=%lu\r\n", (unsigned long)ins_stats.frame_count,
              (unsigned long)ins_stats.gap_count, (unsigned long)ins_stats.stale_accel_count,
              (unsigned long)ins_stats.reject_count, (unsigned long)ins_stats.timeout_count);
    shell_out("  driver: gyro=%lu accel=%lu temp=%lu overrun(g/a)=%lu/%lu spi_err=%lu start_err=%lu recover=%lu\r\n",
              (unsigned long)driver_stats.gyro_count, (unsigned long)driver_stats.accel_count,
              (unsigned long)driver_stats.temperature_count, (unsigned long)driver_stats.gyro_overrun,
              (unsigned long)driver_stats.accel_overrun, (unsigned long)driver_stats.spi_error_count,
              (unsigned long)driver_stats.start_error_count, (unsigned long)driver_stats.recover_count);
    shell_out("  event: posted=%lu handled=%lu dropped=%lu\r\n", (unsigned long)ins_task.posted, (unsigned long)ins_task.handled,
              (unsigned long)ins_task.dropped);
    return 0;
}
SHELL_CMD(ins_info, shell_ins_info, show_attitude_and_statistics)

static int shell_ins_imu(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    shell_out("imu present=%d\r\n", (int)bmi088_present());
    print_milli("gyro_x", last_sample.gyro_rad_s[0], "rad/s");
    print_milli("gyro_y", last_sample.gyro_rad_s[1], "rad/s");
    print_milli("gyro_z", last_sample.gyro_rad_s[2], "rad/s");
    print_milli("accel_x", last_sample.accel_m_s2[0], "m/s2");
    print_milli("accel_y", last_sample.accel_m_s2[1], "m/s2");
    print_milli("accel_z", last_sample.accel_m_s2[2], "m/s2");
    return 0;
}
SHELL_CMD(ins_imu, shell_ins_imu, show_latest_calibrated_imu_sample)
#endif