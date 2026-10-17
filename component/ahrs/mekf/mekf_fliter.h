#ifndef MEKF_FLITER_H
#define MEKF_FLITER_H

#include <stdint.h>

/* 可选的 float 缓冲区操作。平台提供 CMSIS-DSP 头文件和库时才开启；
 * 独立算法默认使用通用 C99 实现。 */
#ifndef MEKF_USE_CMSIS_DSP
#define MEKF_USE_CMSIS_DSP 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 使用入口（先读这里，再看 .c 顶部的算法流程与调参顺序）：
 *   MEKF_Init(&filter, q0, NULL, NULL);
 *   MEKF_Update(&filter, gyro_rad_s, accel_m_s2, dt_seconds, flags);
 *   输出 filter.q / filter.bias；状态、拒绝计数、NIS 用于解释波形。
 *   gyro/accel 必须来自同一机体坐标系；驱动减过固定零偏后，bias 只代表
 *   剩余零偏，不得在外部先减 filter.bias 再把结果送进来，否则会扣两次。
 *
 * Hamilton [w,x,y,z], body -> navigation; stationary specific force is +Z.
 * Left/global error: q_true = Exp(delta_theta_nav) * q.
 * Nominal state (q,bias_body), covariance ordering (delta_theta_nav, delta_bias_body).
 * Gravity correction constrains both navigation-Z attitude increments and
 * body-gravity bias increments; it does not create a heading measurement.
 * No HAL, heap or global state. Optional CMSIS-DSP buffer backend above.
 * One owner per instance. */
typedef struct {
    /* 下面 noise 字段都填标准差/噪声密度，内部才平方生成 Q/R。
     * 不兼容旧 QuaternionEKF 的 Q1=10.5、R=1e6 等经验参数。 */
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
    /* 调用方明确声明零角速度，例如板子确实固定静止。
     * 不能只凭陀螺模长小就设置：真实慢速 yaw 转动不能当成零偏。 */
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
    float accel_nis;           /* 归一化创新平方；未进行创新检验时为 -1 */
    float accel_norm;
    float stationary_elapsed;
    float zero_rate_elapsed;
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
/* q/bias 为 NULL 时使用单位四元数/零偏零值，config 为 NULL 时使用默认参数。
 * 配置或初值非法返回 0，初始化失败后实例保持禁用。 */
uint8_t MEKF_Init(MEKF_State *state, const float q[4], const float bias[3],
                  const MEKF_Config *config);
/* gyro 单位 rad/s，accel 单位 m/s^2，dt 单位秒。
 * 陀螺或 dt 非法时整拍跳过；加速度非法时只跳过校正，仍进行陀螺预测。 */
uint8_t MEKF_Update(MEKF_State *state, const float gyro[3], const float accel[3],
                    float dt, uint8_t flags);
/* 可选：独立的导航系 yaw 观测，rad / rad^2；返回 1 表示接受。
 * 固定底座编码器必须先校准安装外参、轴方向、零位和时钟。它不是泛用
 * “电机角=IMU yaw”接口：任意关节角必须先经过运动学转换。
 * 调用方负责新鲜度/仅消费一次/时间对齐；不要把延迟角度直接融合到当前时刻。
 * 初始 yaw 应先对齐到参考附近；此接口为局部 EKF 校正，不用于大角度重置。
 * 不默认接入任何外设；使用它的结果不能再用同一编码器作独立精度真值。 */
uint8_t MEKF_FuseHeading(MEKF_State *state, float yaw_rad, float variance_rad2);
/* 标准 ZYX 欧拉角，单位弧度：roll=X、pitch=Y、yaw=Z。
 * 旧 INS 接口的 roll/pitch 命名相反，在接口边界适配。 */
void MEKF_GetEuler(const MEKF_State *state, float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif
#endif
