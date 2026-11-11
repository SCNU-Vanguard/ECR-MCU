#ifndef MEKF_FLITER_H
#define MEKF_FLITER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 使用入口：
 *   MEKF_Init(&filter, q0, NULL, NULL);
 *   MEKF_Update(&filter, gyro_rad_s, accel_m_s2, dt_seconds, flags);
 * 输入必须使用同一机体系和规定单位。驱动固定标定后的零偏由外部扣除，
 * filter.bias 只表示剩余零偏，不能重复扣除。
 *
 * 四元数为 Hamilton [w,x,y,z]，表示机体系到固定参考坐标系的旋转。
 * 固定参考坐标系表示上电初始化时的板子坐标系；实现上由首帧有效加速度建立，
 * yaw 初值为 0，随后随陀螺积分变化；
 * 静止比力在该坐标系中约定为 +Z。
 * 名义状态为 (q, bias)，协方差顺序为 (姿态误差、零偏误差)。
 * 加速度只约束倾斜，不提供绝对航向。 */
typedef struct {
    /* 噪声字段填写标准差或噪声密度，内部平方后生成 Q/R。 */
    float gyro_noise;          /* 陀螺连续白噪声密度，rad/sqrt(s)，内部平方 */
    float bias_walk;           /* 零偏随机游走密度，(rad/s)/sqrt(s)，内部平方 */
    float accel_noise;         /* 单位加速度方向标准差，无量纲，内部平方为 R */
    float initial_angle_std;   /* 初始姿态误差标准差，rad */
    float initial_bias_std;    /* 驱动标定后剩余零偏的初始标准差，rad/s */
    float gravity;             /* 静止加速度标称模长，m/s^2 */
    float accel_reject;        /* |a|-g 的硬拒绝门限，m/s^2 */
    float accel_adapt;         /* 模长偏差放大 R 的尺度，m/s^2；越小越保守 */
    float accel_nis_limit;     /* 二维 NIS 门限；默认 9.21，理想模型下 99% */
    float max_dt;              /* 最大有效积分周期，秒；超出整拍丢弃 */
    float max_gyro;            /* 每轴角速度上限，rad/s，匹配 BMI088 +/-2000 deg/s */
    float bias_limit;          /* 每轴剩余零偏上限，rad/s；越界拒绝整次校正 */
    float stationary_gyro;     /* ZARU 附加条件：驱动校准后角速度模长，rad/s */
    float stationary_accel;    /* ZARU 附加条件：abs(|a|-g)，m/s^2 */
    float stationary_time;     /* 明确静止标志和附加条件持续满足的时间，秒 */
    float zero_rate_noise;     /* 零角速观测标准差，rad/s；小值意味着更相信静止 */
} MEKF_Config;

enum {
    MEKF_GYRO_CLIPPED = 1U,
    MEKF_ACCEL_CLIPPED = 2U,
    /* 调用的时候使用门限确认静止后才能设置；不能仅凭陀螺模长判定。 */
    MEKF_STATIONARY = 4U
};
enum {
    MEKF_PREDICTED = 1U,
    MEKF_ACCEL_USED = 2U,
    MEKF_ZERO_RATE_USED = 4U,
    MEKF_ACCEL_REJECTED = 8U,
    MEKF_INVALID_SAMPLE = 16U,
    MEKF_HEADING_USED = 32U
};

typedef struct {
    float q[4];
    float bias[3];
    float P[36];               /* 按行存储的 6×6 协方差矩阵 */
    MEKF_Config config;
    float accel_nis;           /* 归一化平方；未进行检验时为 -1 */
    float accel_norm;
    float stationary_elapsed;
    float zero_rate_elapsed;
    float zero_rate_integral[3]; /* 静止窗口内 gyro * dt，避免降采样混叠为零偏 */
    float tilt_recovery_elapsed;
    uint32_t predict_count;
    uint32_t accel_accept_count;
    uint32_t accel_reject_count;
    uint32_t zero_rate_count;
    uint32_t invalid_count;
    uint32_t numeric_fault_count;
    float heading_nis;
    uint32_t heading_accept_count, heading_reject_count;
    uint8_t initialized;
    uint8_t status;
} MEKF_State;

MEKF_Config MEKF_DefaultConfig(void);
/* q、bias 或 config 为 NULL 时使用默认初值；非法参数返回 0。 */
uint8_t MEKF_Init(MEKF_State *state, const float q[4], const float bias[3],
                  const MEKF_Config *config);
/* gyro 为 rad/s，accel 为 m/s^2，dt 为秒。
 * 陀螺或 dt 无效时跳过整拍；加速度无效时仍保留陀螺预测。 */
uint8_t MEKF_Update(MEKF_State *state, const float gyro[3], const float accel[3],
                    float dt, uint8_t flags);
/* 缺测时清除连续静止证据及未完成的零角速观测窗口，保留姿态/零偏。 */
void MEKF_BreakContinuity(MEKF_State *state);
/* 自己配置的固定参考坐标系 yaw 观测，单位为 rad/rad^2；返回 1 表示接受。 */
uint8_t MEKF_FuseHeading(MEKF_State *state, float yaw_rad, float variance_rad2);
/* 输出 ZYX 欧拉角，单位为弧度。 */
void MEKF_GetEuler(const MEKF_State *state, float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif
#endif
