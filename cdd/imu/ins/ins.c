/* 惯导任务。数据源由 ins_config.h 的 INS_IMU 选择。
 * BMI088：DRDY 中断 -> SPI DMA -> 帧回调里 event_post_isr -> 本任务换算、标定、ahrs_update。
 *   超时 100 ms 没有新帧就 bmi088_recover。启用加热时，温度进窗口之前不开始解算。
 * HI05R3：经 HI05R3_BUS 指定的 CAN 总线收 J1939。USE_DEVICE_ATTITUDE 为 0 时由角速度帧驱动 ahrs_update，为 1 时等三类帧齐备后使用模块四元数。
 * shell：ins_info 看姿态和统计，ins_imu 看最近一帧物理量。 */
#include <math.h>
#include <string.h>
#include "osal/event.h"
#include "osal/osal.h"
#include "bsp/cpu/bsp_cpu.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/log/log.h"
#include "component/ahrs/ahrs.h"
#include "cdd/imu/ins/ins.h"
#include "ins_config.h"
#include "bsp/tick/bsp_tick.h"

#if INS_IMU == INS_IMU_BMI088
#include "cdd/imu/bmi088/bmi088.h"
#endif

#ifndef INS_IMU
#error "ins_config.h must define INS_IMU"
#endif
#if INS_IMU != INS_IMU_BMI088 && INS_IMU != INS_IMU_HI05R3
#error "INS_IMU must be INS_IMU_BMI088 or INS_IMU_HI05R3"
#endif
#ifndef HI05R3_USE_DEVICE_ATTITUDE
#error "ins_config.h must define HI05R3_USE_DEVICE_ATTITUDE"
#endif
#if HI05R3_USE_DEVICE_ATTITUDE != 0 && HI05R3_USE_DEVICE_ATTITUDE != 1
#error "HI05R3_USE_DEVICE_ATTITUDE must be 0 or 1"
#endif
#if INS_IMU == INS_IMU_HI05R3
#include "cdd/imu/hi05r3/hi05r3.h"
#ifndef CONFIG_CDD_IMU_HI05R3
#error "INS_IMU is HI05R3 but CONFIG_CDD_IMU_HI05R3 is off"
#endif
#endif

#define INS_PI                      3.14159265f
#define INS_TWO_PI                  6.2831853f

#ifndef INS_ASSUME_STATIONARY
#define INS_ASSUME_STATIONARY 0U
#endif
#if INS_ASSUME_STATIONARY != 0U && INS_ASSUME_STATIONARY != 1U
#error "INS_ASSUME_STATIONARY must be 0 or 1"
#endif

#ifdef CONFIG_CDD_IMU_HEATER
#define INS_TEMP_ENABLE_MIN_C       (HEATER_TARGET_C - 2.0f)
#define INS_TEMP_ENABLE_MAX_C       (HEATER_TARGET_C + 2.0f)
#define INS_TEMP_SETTLE_US          2500000u
#define INS_TEMP_MAX_AGE_US         500000u
#endif

#if INS_IMU == INS_IMU_BMI088
/* 加速度计样本比陀螺仪样本旧过这么多就认为过期，这一拍加速度计不参与校正（只用陀螺积分）：两个加速度计周期 + 1 ms */
#define INS_ACCEL_MAX_AGE_US        (2u * BMI088_ACCEL_SAMPLE_PERIOD_US + 1000u)
#endif

typedef struct
{
    uint32_t frame_count;
    uint32_t gap_count;             /* 陀螺仪序号不连续（丢样本） */
    uint32_t stale_accel_count;
    uint32_t reject_count;          /* ahrs_update 拒绝或失败 */
    uint32_t gyro_only_reject_count;/* 加速度无效、仅用陀螺仪输入时被拒绝的帧数 */
    uint32_t temperature_wait_count;/* 温度样本无效，或加热版尚未达到稳定温度时跳过的帧数 */
    uint32_t temperature_gate_reset_count;
    uint32_t timeout_count;
    float    last_dt_s;
} ins_stats_t;

#if INS_IMU == INS_IMU_BMI088 || (INS_IMU == INS_IMU_HI05R3 && HI05R3_USE_DEVICE_ATTITUDE == 0)
static ahrs_t ahrs;
#endif
static ins_stats_t ins_stats;
#if INS_IMU == INS_IMU_BMI088
static bmi088_sample_t last_sample;
#endif
static ins_attitude_t published_attitude;
static volatile uint32_t publish_version;                  /* 奇数 = 正在写，用来让读者拿到一致的快照 */
static bool have_previous_frame;
static uint32_t previous_gyro_timestamp_us;
static uint32_t previous_gyro_sequence;
#if INS_IMU == INS_IMU_BMI088
static bool ins_temperature_gate_open;
static bool ins_temperature_waiting;
static uint32_t ins_temperature_ready_since_us;
static uint32_t ins_temperature_last_sequence;
static uint32_t ins_temperature_last_update_us;
#endif
#if INS_IMU == INS_IMU_HI05R3 && HI05R3_USE_DEVICE_ATTITUDE
static bool device_yaw_ready;
static float device_last_yaw;
#endif

/* 在 publish_version 写入区间内调用，只清除姿态，不影响传感器快照。 */
static void clear_attitude(void)
{
    published_attitude.valid = false;
    memset(published_attitude.quaternion, 0, sizeof published_attitude.quaternion);
    published_attitude.roll_rad = 0.0f;
    published_attitude.pitch_rad = 0.0f;
    published_attitude.yaw_rad = 0.0f;
    published_attitude.yaw_total_rad = 0.0f;
    published_attitude.update_count = 0u;
    published_attitude.ins_cost_time = 0u;
}

static void invalidate_attitude(void)
{
    publish_version++;
    DMB();
    clear_attitude();
    DMB();
    publish_version++;
}

#if INS_IMU == INS_IMU_BMI088
/* ---- 陀螺零偏标定：静止采集 INS_CALI_DURATION_S 秒，在本任务里累加（和姿态解算共用同一帧数据）----
 * 新零偏 = 当前表里的零偏 + 采集期间"已扣除当前零偏的角速度"的平均值。
 * 只有陀螺零偏能靠静止一次标出来；加速度计 scale/offset 需要多个姿态，这里原样保留。 */
#define INS_CALI_DURATION_S         60u
#define INS_CALI_FRAME_TARGET       (INS_CALI_DURATION_S * BMI088_RATE_HZ)
#define INS_CALI_MOVING_LIMIT_RAD_S 0.1f           /* 任一轴峰峰值超过它，认为采集期间动过 */

typedef enum
{
    INS_CALI_IDLE = 0,
    INS_CALI_RUNNING,
    INS_CALI_DONE
} ins_cali_state_e;

typedef struct
{
    volatile ins_cali_state_e state;
    uint32_t frame_count;
    double   gyro_sum[3];
    double   temperature_sum;
    float    gyro_min[3];
    float    gyro_max[3];
} ins_cali_t;

static ins_cali_t cali;

static void calibration_start(void)
{
    memset(&cali, 0, sizeof cali);
    for (int axis = 0; axis < 3; axis++)
    {
        cali.gyro_min[axis] = 1.0e9f;
        cali.gyro_max[axis] = -1.0e9f;
    }
    cali.state = INS_CALI_RUNNING;                          /* 最后置状态，ins_task 看到 RUNNING 时数据已清零 */
}

static void calibration_feed(const bmi088_sample_t *sample)
{
    if (cali.state != INS_CALI_RUNNING)
    {
        return;
    }
    for (int axis = 0; axis < 3; axis++)
    {
        float value = sample->gyro_rad_s[axis];

        cali.gyro_sum[axis] += (double)value;
        if (value < cali.gyro_min[axis])
        {
            cali.gyro_min[axis] = value;
        }
        if (value > cali.gyro_max[axis])
        {
            cali.gyro_max[axis] = value;
        }
    }
    cali.temperature_sum += (double)sample->temperature_c;
    cali.frame_count++;
    if (cali.frame_count >= INS_CALI_FRAME_TARGET)
    {
        cali.state = INS_CALI_DONE;
    }
}
#endif

#if INS_IMU == INS_IMU_BMI088 || (INS_IMU == INS_IMU_HI05R3 && HI05R3_USE_DEVICE_ATTITUDE == 0)
/* 发布一帧传感器快照，和需要时更新姿态。和具体数据源无关：传感器量由调用方给。 */
static void publish_snapshot(const float gyro_rad_s[3], const float accel_m_s2[3],
                             float temperature_c, bool temperature_valid,
                             bool clear_pose, bool update_pose, uint32_t frame_begin_cycles)
{
    publish_version++;
    DMB();
    memcpy(published_attitude.gyro_rad_s, gyro_rad_s, sizeof published_attitude.gyro_rad_s);
    memcpy(published_attitude.accel_m_s2, accel_m_s2, sizeof published_attitude.accel_m_s2);
    published_attitude.temperature_c = temperature_c;
    published_attitude.temperature_valid = temperature_valid;
    published_attitude.imu_frame_count++;
    if (clear_pose)
    {
        clear_attitude();
    }
    else if (update_pose)
    {
        published_attitude.valid = ahrs.update_count > 0u;
        memcpy(published_attitude.quaternion, ahrs.quaternion, sizeof published_attitude.quaternion);
        published_attitude.roll_rad = ahrs.roll_rad;
        published_attitude.pitch_rad = ahrs.pitch_rad;
        published_attitude.yaw_rad = ahrs.yaw_rad;
        published_attitude.yaw_total_rad = ahrs.yaw_total_rad;
        published_attitude.update_count = ahrs.update_count;
        /* 姿态输出已写完再取终点；无符号周期差支持一次 CYCCNT 回绕，区间须短于一个计数周期。
         * 与 Embedded_Frame 的 last_us 一致：取数前到成功输出，区间内抢占/中断仍计入。 */
        const uint32_t frame_end_cycles = bsp_cycle_count();
        published_attitude.ins_cost_time = (uint32_t)((uint64_t)(frame_end_cycles - frame_begin_cycles) * 1000000u / bsp_cycle_freq_hz());
    }
    DMB();
    publish_version++;
}
#endif

void ins_get_attitude(ins_attitude_t *attitude)
{
    uint32_t version_before;
    uint32_t version_after;

    do
    {
        version_before = publish_version;
        DMB();
        *attitude = published_attitude;
        DMB();
        version_after = publish_version;
    } while ((version_before & 1u) != 0u || version_before != version_after);
}

#if INS_IMU == INS_IMU_HI05R3
#if HI05R3_USE_DEVICE_ATTITUDE
/* 归一化四元数 [w, x, y, z]。模长不合理（0.5..2）返回 false。 */
static bool quat_normalize(const float input[4], float output[4])
{
    float norm2 = 0.0f;

    for (uint32_t index = 0u; index < 4u; index++)
    {
        norm2 += input[index] * input[index];
    }
    if (!(norm2 > 0.25f && norm2 < 4.0f))
    {
        return false;
    }

    const float inv_norm = 1.0f / sqrtf(norm2);

    for (uint32_t index = 0u; index < 4u; index++)
    {
        output[index] = input[index] * inv_norm;
    }
    return true;
}

/* 单位四元数转 ZYX 欧拉角，和 component/ahrs 的约定一致。 */
static void quat_to_euler(const float quat[4], float *roll_rad, float *pitch_rad, float *yaw_rad)
{
    const float w = quat[0];
    const float x = quat[1];
    const float y = quat[2];
    const float z = quat[3];
    float sin_pitch = 2.0f * (w * y - z * x);

    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }
    *roll_rad = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    *pitch_rad = asinf(sin_pitch);
    *yaw_rad = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

/* yaw 增量折到 (-pi, pi]，用来累计连续 yaw。 */
static float yaw_step_wrap(float step_rad)
{
    if (step_rad > INS_PI)
    {
        return step_rad - INS_TWO_PI;
    }
    if (step_rad < -INS_PI)
    {
        return step_rad + INS_TWO_PI;
    }
    return step_rad;
}

static void publish_device_attitude(const hi05r3_frame_t *frame)
{
    float quat[4];
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
    const bool attitude_valid = frame->attitude_valid && quat_normalize(frame->quat, quat);

    if (attitude_valid)
    {
        quat_to_euler(quat, &roll_rad, &pitch_rad, &yaw_rad);
    }

    /* 写入期间 publish_version 为奇数，读者据此丢弃不一致的快照。 */
    publish_version++;
    DMB();
    if (!attitude_valid)
    {
        clear_attitude();
        device_yaw_ready = false;
    }
    memcpy(published_attitude.gyro_rad_s, frame->gyro_rad_s, sizeof published_attitude.gyro_rad_s);
    memcpy(published_attitude.accel_m_s2, frame->accel_m_s2, sizeof published_attitude.accel_m_s2);
    published_attitude.temperature_c = frame->temperature_c;
    published_attitude.temperature_valid = frame->temperature_valid;
    published_attitude.imu_frame_count++;
    if (attitude_valid)
    {
        published_attitude.valid = true;
        memcpy(published_attitude.quaternion, quat, sizeof published_attitude.quaternion);
        published_attitude.roll_rad = roll_rad;
        published_attitude.pitch_rad = pitch_rad;
        published_attitude.yaw_rad = yaw_rad;
        if (device_yaw_ready)
        {
            published_attitude.yaw_total_rad += yaw_step_wrap(yaw_rad - device_last_yaw);
        }
        else
        {
            published_attitude.yaw_total_rad = yaw_rad;
            device_yaw_ready = true;
        }
        device_last_yaw = yaw_rad;
        published_attitude.update_count++;
        published_attitude.ins_cost_time = frame->communication_time_us;
    }
    DMB();
    publish_version++;
}
#endif

static void ins_run(void *argument, const uint8_t *data, uint16_t length)
{
    hi05r3_frame_t frame;

    (void)argument;
    if (data == NULL)
    {
        ins_stats.timeout_count++;
        have_previous_frame = false;
#if HI05R3_USE_DEVICE_ATTITUDE
        device_yaw_ready = false;
#else
        ahrs_break_continuity(&ahrs);                       /* 链路断过，不能把缺口当连续数据 */
#endif
        invalidate_attitude();
        return;
    }
    if (length != sizeof frame)
    {
        return;
    }
#if HI05R3_USE_DEVICE_ATTITUDE == 0
    const uint32_t frame_begin_cycles = bsp_cycle_count();
#endif
    memcpy(&frame, data, sizeof frame);
    ins_stats.frame_count++;

    float dt_s = 1.0f / (float)HI05R3_RATE_HZ;

    if (have_previous_frame)
    {
        dt_s = (float)(uint32_t)(frame.timestamp_us - previous_gyro_timestamp_us) * 1.0e-6f;
        if ((uint32_t)(frame.sequence - previous_gyro_sequence) != 1u)
        {
            ins_stats.gap_count++;
#if HI05R3_USE_DEVICE_ATTITUDE == 0
            ahrs_break_continuity(&ahrs);
#endif
        }
    }
    have_previous_frame = true;
    previous_gyro_timestamp_us = frame.timestamp_us;
    previous_gyro_sequence = frame.sequence;
    ins_stats.last_dt_s = dt_s;
#if HI05R3_USE_DEVICE_ATTITUDE
    publish_device_attitude(&frame);
#else
    ahrs_sample_t ahrs_sample;

    memcpy(ahrs_sample.gyro_rad_s, frame.gyro_rad_s, sizeof ahrs_sample.gyro_rad_s);
    memcpy(ahrs_sample.accel_m_s2, frame.accel_m_s2, sizeof ahrs_sample.accel_m_s2);
    ahrs_sample.dt_s = dt_s;
    ahrs_sample.gyro_clipped = frame.gyro_clipped;
    ahrs_sample.accel_clipped = frame.accel_clipped || !frame.accel_fresh;
    ahrs_sample.stationary = (INS_ASSUME_STATIONARY != 0U);
    if (!frame.accel_fresh)
    {
        ins_stats.stale_accel_count++;
    }
    bool gyro_only_update = ahrs_sample.accel_clipped;
    int32_t update_result = ahrs_update(&ahrs, &ahrs_sample);

    if (update_result != ERR_OK && update_result != ERR_BUSY)
    {
        ins_stats.reject_count++;
        if (gyro_only_update)
        {
            ins_stats.gyro_only_reject_count++;
        }
    }
    publish_snapshot(frame.gyro_rad_s, frame.accel_m_s2, frame.temperature_c, frame.temperature_valid,
                     false, update_result == ERR_OK, frame_begin_cycles);
#endif
}
/* 200 Hz 下 16 帧约 80 ms。100 ms 没有角速度报文就认为链路断了。 */
EVENT_TASK_DEFINE(ins_task, OS_PRIORITY_HIGH, OS_STACK_LARGE, EVENT_QUEUE, sizeof(hi05r3_frame_t), OS_QUEUE_LEN_16, OS_TIMEOUT_100MS, ins_run, NULL);

static void ins_on_frame(const hi05r3_frame_t *frame, void *context)
{
    (void)context;
    (void)event_post_isr(&ins_task, frame, sizeof *frame);
}

static int32_t ins_init(void)
{
#if HI05R3_USE_DEVICE_ATTITUDE == 0
    int32_t ret = ahrs_start(&ahrs, INS_ALGORITHM, 1.0f / (float)HI05R3_RATE_HZ);

    if (ret != ERR_OK)
    {
        LOGE("default algorithm unavailable: %d", (int)ret);
        return ret;
    }
#endif
    hi05r3_set_frame_callback(ins_on_frame, NULL);
    return ERR_OK;
}
#else
/* 温度是否允许解算。启用加热：温度样本要新，并且在目标温度窗口内。
 * 没启用加热：不依赖温度寄存器，直接允许；这时零偏要按不加热的状态重新标定。 */
static bool temperature_in_window(const bmi088_frame_t *frame)
{
#ifdef CONFIG_CDD_IMU_HEATER
    const float temperature_c = 0.125f * (float)frame->temperature_raw + 23.0f;

    return frame->temperature_sequence > 0u &&
           (uint32_t)(frame->gyro_timestamp_us - ins_temperature_last_update_us) <= INS_TEMP_MAX_AGE_US &&
           temperature_c >= INS_TEMP_ENABLE_MIN_C && temperature_c <= INS_TEMP_ENABLE_MAX_C;
#else
    (void)frame;
    return true;
#endif
}

/* 温度进窗口后还要稳定 INS_TEMP_SETTLE_US 才开闸门。没启用加热时不用等。 */
static bool temperature_settled(uint32_t gyro_timestamp_us)
{
#ifdef CONFIG_CDD_IMU_HEATER
    return (uint32_t)(gyro_timestamp_us - ins_temperature_ready_since_us) >= INS_TEMP_SETTLE_US;
#else
    (void)gyro_timestamp_us;
    return true;
#endif
}
static void ins_run(void *argument, const uint8_t *data, uint16_t length)
{
    bmi088_frame_t frame;
    ahrs_sample_t ahrs_sample;

    (void)argument;
    if (data == NULL)
    {
        ins_stats.timeout_count++;
        /* 没有帧只说明数据流断了，不说明温度离开了窗口：只打断连续性、让姿态无效，不动温度闸门。
         * 闸门只在温度出窗或温度序号停更时关闭（见 temperature_in_window），恢复来帧后不必再等 INS_TEMP_SETTLE_US。
         * 闸门还没开（首次上电稳温中）时仍重新计稳温时间，保持首次稳温行为 */
        have_previous_frame = false;
        ahrs_break_continuity(&ahrs);
        if (!ins_temperature_gate_open)
        {
            ins_temperature_waiting = false;
        }
        invalidate_attitude();
        (void)bmi088_recover();
        return;
    }
    if (length != sizeof frame)
    {
        return;
    }
    /* 从本任务复制已收到的原始帧之前计时，包含快照、换算/标定、检查、解算和输出。
     * 不回溯到 DRDY，也不包含此前的 SPI DMA、事件入队/出队或调度等待。 */
    const uint32_t frame_begin_cycles = bsp_cycle_count();
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
    if (frame.temperature_sequence != ins_temperature_last_sequence)
    {
        ins_temperature_last_sequence = frame.temperature_sequence;
        ins_temperature_last_update_us = frame.gyro_timestamp_us;
    }
    if (temperature_in_window(&frame))
    {
        if (!ins_temperature_gate_open)
        {
            if (!ins_temperature_waiting)
            {
                ins_temperature_waiting = true;
                ins_temperature_ready_since_us = frame.gyro_timestamp_us;
            }
            if (temperature_settled(frame.gyro_timestamp_us))
            {
                ins_temperature_gate_open = true;
                ins_temperature_waiting = false;
                have_previous_frame = false;
                (void)ahrs_start(&ahrs, INS_ALGORITHM, 1.0f / (float)BMI088_RATE_HZ);
            }
        }
    }
    else
    {
        if (ins_temperature_gate_open)
        {
            ins_stats.temperature_gate_reset_count++;
        }
        ins_temperature_gate_open = false;
        ins_temperature_waiting = false;
    }
    if (!ins_temperature_gate_open)
    {
        have_previous_frame = false;
        ins_stats.temperature_wait_count++;
        publish_snapshot(last_sample.gyro_rad_s, last_sample.accel_m_s2, last_sample.temperature_c,
                         frame.temperature_sequence > 0u, true, false, frame_begin_cycles);
        return;
    }
    calibration_feed(&last_sample);
    memcpy(ahrs_sample.gyro_rad_s, last_sample.gyro_rad_s, sizeof ahrs_sample.gyro_rad_s);
    memcpy(ahrs_sample.accel_m_s2, last_sample.accel_m_s2, sizeof ahrs_sample.accel_m_s2);
    ahrs_sample.dt_s = dt_s;
    ahrs_sample.gyro_clipped = last_sample.gyro_clipped;
    ahrs_sample.accel_clipped = last_sample.accel_clipped;
    ahrs_sample.stationary = (INS_ASSUME_STATIONARY != 0U);

    if ((uint32_t)(frame.gyro_timestamp_us - frame.accel_timestamp_us) > INS_ACCEL_MAX_AGE_US)
    {
        ins_stats.stale_accel_count++;
        ahrs_sample.accel_clipped = true;                   /* 借用"加速度计不可信"标志：这一拍只积分陀螺 */
    }
    bool gyro_only_update = ahrs_sample.accel_clipped;

    int32_t update_result = ahrs_update(&ahrs, &ahrs_sample);

    if (update_result != ERR_OK)
    {
        /* 未成功输出时保留上一有效帧的姿态及耗时，避免两者对应不同帧。 */
        if (update_result != ERR_BUSY)
        {
            ins_stats.reject_count++;
            if (gyro_only_update)
            {
                ins_stats.gyro_only_reject_count++;
            }
        }
    }
    publish_snapshot(last_sample.gyro_rad_s, last_sample.accel_m_s2, last_sample.temperature_c,
                     true, false, update_result == ERR_OK, frame_begin_cycles);
}
/* 任务名 ins_task：事件驱动，高优先级，大栈（滤波算法的局部矩阵较多）。
 * OS_QUEUE_LEN_16：2 kHz 下约 8 ms 缓冲。OS_TIMEOUT_100MS：这么久没有新帧就重新拉起采集 */
EVENT_TASK_DEFINE(ins_task, OS_PRIORITY_HIGH, OS_STACK_LARGE, EVENT_QUEUE, sizeof(bmi088_frame_t), OS_QUEUE_LEN_16, OS_TIMEOUT_100MS, ins_run, NULL);

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
        LOGE("default algorithm unavailable: %d", (int)ret);
        return ret;
    }
    if (!bmi088_present())
    {
        LOGW("bmi088 not present, ins disabled");
        return ERR_NODEV;
    }
    bmi088_set_frame_callback(ins_on_frame, NULL);
    return bmi088_start();
}
#endif
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

    (void)argc;
    (void)argv;
    ins_get_attitude(&attitude);
#if INS_IMU == INS_IMU_HI05R3 && HI05R3_USE_DEVICE_ATTITUDE
    shell_out("ins: algorithm=hi05r3 valid=%d updates=%lu\r\n", (int)attitude.valid, (unsigned long)attitude.update_count);
#else
    shell_out("ins: algorithm=%s valid=%d updates=%lu\r\n", ahrs_algorithm_name(ahrs.algorithm), (int)attitude.valid,
              (unsigned long)attitude.update_count);
#endif
    print_milli("roll", attitude.roll_rad * RAD_TO_DEG, "deg");
    print_milli("pitch", attitude.pitch_rad * RAD_TO_DEG, "deg");
    print_milli("yaw", attitude.yaw_rad * RAD_TO_DEG, "deg");
    print_milli("yaw_total", attitude.yaw_total_rad * RAD_TO_DEG, "deg");
    if (attitude.temperature_valid)
    {
        print_milli("temperature", attitude.temperature_c, "C");
    }
    else
    {
        shell_out("  temperature = unavailable\r\n");
    }
    print_milli("dt", ins_stats.last_dt_s * 1000.0f, "ms");
#if INS_IMU == INS_IMU_HI05R3 && HI05R3_USE_DEVICE_ATTITUDE
    shell_out("  ins_cost_time=%lu us (CAN frame receive span)\r\n", (unsigned long)attitude.ins_cost_time);
#else
    shell_out("  ins_cost_time=%lu us (task snapshot -> output)\r\n", (unsigned long)attitude.ins_cost_time);
#endif
    shell_out("  imu_frames=%lu frames=%lu gaps=%lu\r\n", (unsigned long)attitude.imu_frame_count,
              (unsigned long)ins_stats.frame_count, (unsigned long)ins_stats.gap_count);
#if INS_IMU == INS_IMU_BMI088
    shell_out("  temp_wait=%lu gate_resets=%lu\r\n", (unsigned long)ins_stats.temperature_wait_count,
              (unsigned long)ins_stats.temperature_gate_reset_count);
#endif
    shell_out("  stale_accel=%lu rejects=%lu gyro_only_rejects=%lu timeouts=%lu\r\n",
              (unsigned long)ins_stats.stale_accel_count,
              (unsigned long)ins_stats.reject_count, (unsigned long)ins_stats.gyro_only_reject_count,
              (unsigned long)ins_stats.timeout_count);
#if INS_IMU == INS_IMU_HI05R3
    {
        hi05r3_stats_t driver_stats;

        hi05r3_get_stats(&driver_stats);
        shell_out("  driver: rx=%lu acc=%lu gyro=%lu quat=%lu incli=%lu temp=%lu publish=%lu ignored=%lu\r\n",
                  (unsigned long)driver_stats.rx_frames, (unsigned long)driver_stats.acc_count,
                  (unsigned long)driver_stats.gyro_count,
                  (unsigned long)driver_stats.quat_count, (unsigned long)driver_stats.inclinometer_count,
                  (unsigned long)driver_stats.temperature_count,
                  (unsigned long)driver_stats.publish_count, (unsigned long)driver_stats.ignored_count);
        float inclinometer_x_rad;

        if (hi05r3_get_inclinometer(&inclinometer_x_rad))
        {
            print_milli("inclinometer_x", inclinometer_x_rad * RAD_TO_DEG, "deg");
        }
        else
        {
            shell_out("  inclinometer_x = unavailable\r\n");
        }
    }
#else
    {
        bmi088_stats_t driver_stats;

        bmi088_get_stats(&driver_stats);
        shell_out("  driver: gyro=%lu accel=%lu temp=%lu overrun(g/a)=%lu/%lu spi_err=%lu start_err=%lu recover=%lu\r\n",
                  (unsigned long)driver_stats.gyro_count, (unsigned long)driver_stats.accel_count,
                  (unsigned long)driver_stats.temperature_count, (unsigned long)driver_stats.gyro_overrun,
                  (unsigned long)driver_stats.accel_overrun, (unsigned long)driver_stats.spi_error_count,
                  (unsigned long)driver_stats.start_error_count, (unsigned long)driver_stats.recover_count);
    }
#endif
    shell_out("  event: posted=%lu handled=%lu dropped=%lu\r\n", (unsigned long)ins_task.posted, (unsigned long)ins_task.handled,
              (unsigned long)ins_task.dropped);
    return 0;
}
SHELL_CMD(ins_info, shell_ins_info, show_attitude_and_statistics)

#if INS_IMU == INS_IMU_BMI088
/* 小数按固定 10 位打印（标定表需要这个精度，C 库没开浮点 printf） */
static void format_decimal10(char *text, size_t size, double value)
{
    int64_t scaled = (int64_t)(value * 1.0e10 + (value >= 0.0 ? 0.5 : -0.5));
    const char *sign = (scaled < 0) ? "-" : "";

    if (scaled < 0)
    {
        scaled = -scaled;
    }
    /* long 只有 32 位，10 位小数的整数（最大 1e10 量级）放不下，拆成两段 5 位打印 */
    uint32_t integer_part = (uint32_t)(scaled / 10000000000LL);
    uint32_t fraction = (uint32_t)(scaled % 10000000000LL / 100000LL);
    uint32_t fraction_low = (uint32_t)(scaled % 100000LL);

    snprintf(text, size, "%s%lu.%05lu%05lu", sign, (unsigned long)integer_part, (unsigned long)fraction, (unsigned long)fraction_low);
}
#endif

/* ins_cali：开始静止采集 INS_CALI_DURATION_S 秒（命令会一直等到采完），然后给出标定表。采集期间板子必须静止，最好等加热到恒温之后。
 * 命令在 shell 任务里阻塞，期间其他 shell 命令不可用；3 秒收不到 IMU 新帧就放弃 */
static int shell_ins_cali(int argc, char *argv[])
{
#if INS_IMU != INS_IMU_BMI088
    (void)argc;
    (void)argv;
    shell_out("ins_cali is only for the BMI088 gyro offset\r\n");
    return -1;
#else
    bmi088_calibration_t current;
    uint32_t last_frame_count = 0u;
    uint32_t stalled_seconds = 0u;

    (void)argc;
    (void)argv;
    if (!bmi088_present())
    {
        shell_out("imu not present\r\n");
        return -1;
    }
    calibration_start();
    shell_out("calibration started: keep the board still for %u s\r\n", (unsigned)INS_CALI_DURATION_S);
    for (uint32_t second = 1u; cali.state == INS_CALI_RUNNING; second++)
    {
        os_delay_ms(1000u);
        if (cali.frame_count == last_frame_count)
        {
            stalled_seconds++;
            if (stalled_seconds >= 3u)
            {
                cali.state = INS_CALI_IDLE;
                shell_out("no imu frames for 3 s (frames=%lu), calibration aborted\r\n", (unsigned long)cali.frame_count);
                return -1;
            }
        }
        else
        {
            stalled_seconds = 0u;
        }
        last_frame_count = cali.frame_count;
        if (second % 10u == 0u)
        {
            shell_out("  %lu / %lu frames\r\n", (unsigned long)cali.frame_count, (unsigned long)INS_CALI_FRAME_TARGET);
        }
    }
    char text[32];
    char accel_scale_text[32];
    char accel_offset_text[32];
    char gyro_offset_text[32];
    static const char axis_names[3] = { 'X', 'Y', 'Z' };
    bool moved = false;

    bmi088_get_calibration(&current);
    for (int axis = 0; axis < 3; axis++)
    {
        if (cali.gyro_max[axis] - cali.gyro_min[axis] > INS_CALI_MOVING_LIMIT_RAD_S)
        {
            moved = true;
        }
    }
    format_decimal10(text, sizeof text, cali.temperature_sum / (double)cali.frame_count);
    shell_out("calibration done: %lu frames, mean temperature %s C\r\n", (unsigned long)cali.frame_count, text);
    for (int axis = 0; axis < 3; axis++)
    {
        format_decimal10(text, sizeof text, cali.gyro_sum[axis] / (double)cali.frame_count);
        shell_out("  gyro_%c residual mean = %s rad/s", axis_names[axis], text);
        format_decimal10(text, sizeof text, (double)(cali.gyro_max[axis] - cali.gyro_min[axis]));
        shell_out(", peak-to-peak = %s rad/s\r\n", text);
    }
    if (moved)
    {
        shell_out("WARNING: gyro moved during sampling (peak-to-peak > 0.1 rad/s), result is not reliable, run ins_cali again\r\n");
    }
    shell_out("paste into ins_config.h:\r\n");
    shell_out("#define BMI088_CALIBRATION_TABLE(X)\r\n");
    for (int axis = 0; axis < 3; axis++)
    {
        double new_offset = (double)current.gyro_offset_rad_s[axis] + cali.gyro_sum[axis] / (double)cali.frame_count;

        format_decimal10(accel_scale_text, sizeof accel_scale_text, (double)current.accel_scale[axis]);
        format_decimal10(accel_offset_text, sizeof accel_offset_text, (double)current.accel_offset[axis]);
        format_decimal10(gyro_offset_text, sizeof gyro_offset_text, new_offset);
        shell_out("    X(%sf, %sf, %sf)%s\r\n", accel_scale_text, accel_offset_text, gyro_offset_text, (axis < 2) ? " \\" : "");
    }
    shell_out("(accel scale/offset are kept from the current table; only the gyro offset is calibrated)\r\n");
    return moved ? -1 : 0;
#endif
}
SHELL_CMD(ins_cali, shell_ins_cali, calibrate_gyro_offset_and_print_table)

static int shell_ins_imu(int argc, char *argv[])
{
    ins_attitude_t attitude;

    (void)argc;
    (void)argv;
    ins_get_attitude(&attitude);
#if INS_IMU == INS_IMU_HI05R3
    shell_out("imu hi05r3 frames=%lu\r\n", (unsigned long)attitude.imu_frame_count);
#else
    shell_out("imu present=%d frames=%lu\r\n", (int)bmi088_present(), (unsigned long)attitude.imu_frame_count);
#endif
    print_milli("gyro_x", attitude.gyro_rad_s[0], "rad/s");
    print_milli("gyro_y", attitude.gyro_rad_s[1], "rad/s");
    print_milli("gyro_z", attitude.gyro_rad_s[2], "rad/s");
    print_milli("accel_x", attitude.accel_m_s2[0], "m/s2");
    print_milli("accel_y", attitude.accel_m_s2[1], "m/s2");
    print_milli("accel_z", attitude.accel_m_s2[2], "m/s2");
    if (attitude.temperature_valid)
    {
        print_milli("temperature", attitude.temperature_c, "C");
    }
    else
    {
        shell_out("  temperature = unavailable\r\n");
    }
    return 0;
}
SHELL_CMD(ins_imu, shell_ins_imu, show_latest_calibrated_imu_sample)
#endif
