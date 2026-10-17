#ifndef IMU_YAW_CALIBRATION_H
#define IMU_YAW_CALIBRATION_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#ifndef IMU_YAW_BLOCKS
#define IMU_YAW_BLOCKS 30U
#endif
#if IMU_YAW_BLOCKS < 10U || IMU_YAW_BLOCKS > 120U
#error "Yaw calibration requires 10..120 one-second blocks"
#endif

/* 调用方必须把电机角度转换成同一导航系的 IMU yaw（方向、零点、减速比、
 * 安装关系）。角度单位 rad；timestamp_us 为实际接收时刻，不能在读取
 * 缓存时改成 now。sequence 每个新电机反馈递增。只支持固定底座 yaw 参考。
 * valid=0、重复或过期都禁止学习，绝不默认“编码器角度为零”。
 * 不要求调用方提供角度不确定度，滤波器内部保留固定非零观测噪声。 */
typedef struct {
    uint64_t timestamp_us;
    uint32_t sequence;
    float yaw_rad;
    uint8_t valid, motion_command;
} IMU_YawEncoder;

typedef struct {
    uint32_t settle_us, max_skew_us; /* 静止 100 ms；编码器/IMU 时间差绝对值 <=10 ms */
    float encoder_span_rad;       /* 整个静止段的角度范围，不只检查相邻两帧 */
    float encoder_std_rad;        /* 内部固定观测噪声，不是电机数据接口参数 */
    float gyro_limit, gyro_step_limit, accel_error_limit, accel_step_limit;
    float bias_std_floor, residual_std_limit, slope_limit, temperature_span_limit;
} IMU_YawCalibrationConfig;

enum {
    IMU_YAW_NO_REFERENCE, IMU_YAW_MOVING, IMU_YAW_SETTLING,
    IMU_YAW_COLLECTING, IMU_YAW_BAD_SAMPLE, IMU_YAW_DUPLICATE, IMU_YAW_BAD_FIT
};
typedef struct {
    /* 导航系 yaw 角速度的剩余偏差；倾斜时不等于机体系 gyro Z bias！ */
    float yaw_rate_bias, slope, bias_std, slope_std, residual_std, lag1;
    float approximate_ci95;       /* 近似 95% 半宽，不声称已验证实际覆盖率 */
    float effective_blocks, span_s, temperature_span;
    uint64_t timestamp_us;
    uint32_t sequence, samples;
    uint8_t usable;
} IMU_YawCalibrationResult;

typedef struct {
    IMU_YawCalibrationConfig config;
    IMU_YawCalibrationResult result;
    uint32_t samples, total_samples, discarded_windows, duplicate_samples;
    uint32_t reference_faults, motion_resets, fit_rejects;
    uint64_t last_us, quiet_since_us, window_start_us, block_start_us;
    uint64_t last_encoder_us;
    uint32_t last_encoder_sequence;
    float anchor, previous_gyro[3], previous_accel[3], previous_rate;
    float time[IMU_YAW_BLOCKS], mean[IMU_YAW_BLOCKS];
    float integral, block_encoder_start, temperature_min, temperature_max, age_s;
    uint16_t block_samples;
    uint8_t blocks, phase, quiet, settled, have_previous, have_encoder;
} IMU_YawCalibration;

void IMU_YawCalibrationInit(IMU_YawCalibration *state);
/* 读取失败/异常 dt 也会破坏连续静止，不能只等下一个好样本才发现。 */
void IMU_YawCalibrationBreak(IMU_YawCalibration *state);
/* 每 10 ms 消费一次同帧 IMU/编码器快照，无硬件读取、无堆、无阻塞。
 * q 为当前 body->world 四元数，用于把 gyro 投影为 Euler yaw 变化率。
 * 100 ms 准入延迟后才采数；运动/过期/饱和/明显扰动清空未完成窗口。
 * 返回 1 仅表示新完成一份约 30 s 的报告，usable 决定是否通过质量门控。
 * fitted bias 不可直接当独立观测回灌 MEKF，也不可直接当 gyro Z bias。
 * INS 仅在新窗口通过后融合一次真实编码器角度，由 MEKF 的角度/bias
 * 交叉协方差估计零偏；拟合用于质量评估，不重复计算同源 gyro 信息。 */
uint8_t IMU_YawCalibrationPush(IMU_YawCalibration *state, uint64_t now_us,
    const float q[4], const float gyro[3], const float accel[3], float temperature,
    uint8_t clipped, const IMU_YawEncoder *encoder);
#ifdef __cplusplus
}
#endif
#endif
