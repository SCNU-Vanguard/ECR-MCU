#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 面向低算力平台改进的 Madgwick AHRS。
 *
 * 陀螺仪输入：rad/s
 * 加速度计输入：m/s^2；若 gravity 使用相同量纲，也可采用其他一致尺度
 * 磁力计输入：量纲不限，但三个轴必须使用相同尺度
 * 四元数约定：[w, x, y, z]
 */

typedef struct
{
	float startup_beta;              /* 启动阶段增益，随后平滑下降到全局 beta */
	float startup_period;            /* 启动增益渐变时长，s */
	float gravity;                   /* 当地重力加速度模长，m/s^2 */
	float accel_lpf_tau;             /* 加速度一阶低通时间常数，s */
	float accel_magnitude_rejection; /* 当 |模长-g| 达到该值时完全拒绝，m/s^2 */
	float accel_direction_rejection; /* 方向误差达到该角度时完全拒绝，rad */
	float accel_recovery_tau;        /* 加速度置信度平滑恢复时间常数，s */
	float accel_recovery_period;     /* 持续方向拒绝后进入恢复的等待时间，s */
	float rest_gyro_threshold;       /* 去除零偏后的静止角速度阈值，rad/s */
	float rest_accel_threshold;      /* 静止时允许的 |模长-g|，m/s^2 */
	float rest_min_time;             /* 开始学习零偏前必须连续静止的时间，s */
	float bias_tau;                  /* 静止零偏学习时间常数，s */
	float bias_limit;                /* 单轴残余零偏绝对值上限，rad/s */
	float max_dt;                    /* 超过该采样周期的数据直接拒绝，s */
} MadgwickAHRSSettings_t;

typedef struct
{
	float gyro_bias[3];             /* 在线学习的三轴陀螺仪残余零偏 */
	float accel_filtered[3];        /* 用于重力判断的低通加速度 */
	float accel_confidence;         /* 当前加速度修正权重，范围 0~1 */
	float accel_magnitude_error;    /* 加速度模长与 gravity 的差 */
	float accel_direction_error;    /* 实测与预测重力方向误差，rad 近似量 */
	float rest_time;                /* 连续满足静止条件的时间 */
	float rejection_time;           /* 连续拒绝加速度方向的时间 */
	float elapsed_time;             /* 已成功积分的总时间 */
	uint32_t update_count;          /* 成功更新次数 */
	uint32_t accel_reject_count;    /* 加速度被完全忽略的次数 */
	uint32_t invalid_update_count;  /* 非法输入或归一化失败次数 */
	uint8_t stationary;             /* 已达到连续静止判据 */
	uint8_t accel_ignored;          /* 本次不使用加速度修正 */
	uint8_t accel_initialized;      /* 加速度低通状态已经初始化 */
} MadgwickAHRSState_t;

extern volatile float beta;
extern volatile float madg_q0;
extern volatile float madg_q1;
extern volatile float madg_q2;
extern volatile float madg_q3;

extern MadgwickAHRSSettings_t MadgwickSettings;
extern MadgwickAHRSState_t MadgwickState;

void MadgwickAHRSinit(const float quaternion[4]);
void MadgwickAHRSreset(void);
void MadgwickAHRSsetSettings(const MadgwickAHRSSettings_t *settings);
void MadgwickAHRSgetQuaternion(float quaternion[4]);
void MadgwickAHRSgetEuler(float *yaw, float *pitch, float *roll);

/* 推荐接口：显式传入实测采样周期 dt。 */
void MadgwickAHRSupdateDt(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt);
void MadgwickAHRSupdateIMUDt(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float dt);

/* 向后兼容接口：固定按 1 kHz（dt=1 ms）更新。 */
void MadgwickAHRSupdate(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz,
                           float ax, float ay, float az);

float madg_invSqrt(float x);

#ifdef __cplusplus
}
#endif

#endif
