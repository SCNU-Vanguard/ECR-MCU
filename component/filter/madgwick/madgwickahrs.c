#include "component/filter/madgwick/madgwickahrs.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#define MADGWICK_DEFAULT_SAMPLE_PERIOD (0.001f)
#define MADGWICK_DEFAULT_BETA          (0.10f)
#define MADGWICK_NORM_EPSILON          (1.0e-12f)
#define MADGWICK_PI                    (3.14159265358979323846f)

volatile float beta = MADGWICK_DEFAULT_BETA;
volatile float madg_q0 = 1.0f;
volatile float madg_q1 = 0.0f;
volatile float madg_q2 = 0.0f;
volatile float madg_q3 = 0.0f;

MadgwickAHRSSettings_t MadgwickSettings = {
	.startup_beta = 0.40f,
	.startup_period = 3.0f,
	.gravity = 9.81f,
	.accel_lpf_tau = 0.0053f,              /* 1 kHz 采样时截止频率约 30 Hz */
	.accel_magnitude_rejection = 2.0f,
	.accel_direction_rejection = 15.0f * MADGWICK_PI / 180.0f,
	.accel_recovery_tau = 0.25f,
	.accel_recovery_period = 5.0f,
	.rest_gyro_threshold = 3.0f * MADGWICK_PI / 180.0f,
	.rest_accel_threshold = 0.5f,
	.rest_min_time = 1.5f,
	.bias_tau = 10.0f,
	.bias_limit = 0.10f,
	.max_dt = 0.02f,
};

MadgwickAHRSState_t MadgwickState = {
	.accel_confidence = 1.0f,
};

static float madg_clampf(float value, float minimum, float maximum)
{
	if (value < minimum)
	{
		return minimum;
	}
	if (value > maximum)
	{
		return maximum;
	}
	return value;
}

static uint8_t madg_isFinite(float value)
{
	return (uint8_t)((value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX));
}

static uint8_t madg_vectorIsFinite(float x, float y, float z)
{
	return (uint8_t)(madg_isFinite(x) && madg_isFinite(y) && madg_isFinite(z));
}

static float madg_smoothstep(float value)
{
	value = madg_clampf(value, 0.0f, 1.0f);
	return value * value * (3.0f - 2.0f * value);
}

float madg_invSqrt(float x)
{
	if ((!madg_isFinite(x)) || (x <= MADGWICK_NORM_EPSILON))
	{
		return 0.0f;
	}
	return 1.0f / sqrtf(x);
}

static uint8_t madg_normaliseQuaternion(float q0, float q1, float q2, float q3)
{
	/* 先在局部变量中校验和归一化，成功后才原子式发布新的全局四元数。 */
	const float reciprocal_norm = madg_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

	if (reciprocal_norm == 0.0f)
	{
		return 0U;
	}

	q0 *= reciprocal_norm;
	q1 *= reciprocal_norm;
	q2 *= reciprocal_norm;
	q3 *= reciprocal_norm;

	if ((!madg_vectorIsFinite(q0, q1, q2)) || (!madg_isFinite(q3)))
	{
		return 0U;
	}

	madg_q0 = q0;
	madg_q1 = q1;
	madg_q2 = q2;
	madg_q3 = q3;
	return 1U;
}

void MadgwickAHRSreset(void)
{
	/* 恢复单位姿态，并清空滤波、零偏学习和诊断状态。 */
	uint8_t i;

	madg_q0 = 1.0f;
	madg_q1 = 0.0f;
	madg_q2 = 0.0f;
	madg_q3 = 0.0f;

	for (i = 0U; i < 3U; i++)
	{
		MadgwickState.gyro_bias[i] = 0.0f;
		MadgwickState.accel_filtered[i] = 0.0f;
	}

	MadgwickState.accel_confidence = 1.0f;
	MadgwickState.accel_magnitude_error = 0.0f;
	MadgwickState.accel_direction_error = 0.0f;
	MadgwickState.rest_time = 0.0f;
	MadgwickState.rejection_time = 0.0f;
	MadgwickState.elapsed_time = 0.0f;
	MadgwickState.update_count = 0U;
	MadgwickState.accel_reject_count = 0U;
	MadgwickState.invalid_update_count = 0U;
	MadgwickState.stationary = 0U;
	MadgwickState.accel_ignored = 0U;
	MadgwickState.accel_initialized = 0U;
}

void MadgwickAHRSinit(const float quaternion[4])
{
	/* 非法初始四元数回退到单位四元数，避免 NaN 进入后续积分。 */
	MadgwickAHRSreset();

	if (quaternion == NULL)
	{
		return;
	}

	if ((!madg_isFinite(quaternion[0])) ||
	    (!madg_isFinite(quaternion[1])) ||
	    (!madg_isFinite(quaternion[2])) ||
	    (!madg_isFinite(quaternion[3])) ||
	    (!madg_normaliseQuaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])))
	{
		madg_q0 = 1.0f;
		madg_q1 = 0.0f;
		madg_q2 = 0.0f;
		madg_q3 = 0.0f;
		MadgwickState.invalid_update_count++;
	}
}

void MadgwickAHRSsetSettings(const MadgwickAHRSSettings_t *settings)
{
	/* 先复制用户配置，再逐项把非有限值和越界值恢复为安全默认值。 */
	if (settings == NULL)
	{
		return;
	}

	MadgwickSettings = *settings;

	if ((!madg_isFinite(MadgwickSettings.startup_beta)) || (MadgwickSettings.startup_beta < 0.0f))
	{
		MadgwickSettings.startup_beta = 0.40f;
	}
	if ((!madg_isFinite(MadgwickSettings.startup_period)) || (MadgwickSettings.startup_period < 0.0f))
	{
		MadgwickSettings.startup_period = 3.0f;
	}
	if ((!madg_isFinite(MadgwickSettings.gravity)) || (MadgwickSettings.gravity <= 0.0f))
	{
		MadgwickSettings.gravity = 9.81f;
	}
	if ((!madg_isFinite(MadgwickSettings.accel_lpf_tau)) || (MadgwickSettings.accel_lpf_tau < 0.0f))
	{
		MadgwickSettings.accel_lpf_tau = 0.0053f;
	}
	if ((!madg_isFinite(MadgwickSettings.accel_magnitude_rejection)) ||
	    (MadgwickSettings.accel_magnitude_rejection <= 0.0f))
	{
		MadgwickSettings.accel_magnitude_rejection = 2.0f;
	}
	if ((!madg_isFinite(MadgwickSettings.accel_direction_rejection)) ||
	    (MadgwickSettings.accel_direction_rejection <= 0.0f))
	{
		MadgwickSettings.accel_direction_rejection = 15.0f * MADGWICK_PI / 180.0f;
	}
	if ((!madg_isFinite(MadgwickSettings.accel_recovery_tau)) ||
	    (MadgwickSettings.accel_recovery_tau < 0.0f))
	{
		MadgwickSettings.accel_recovery_tau = 0.25f;
	}
	if ((!madg_isFinite(MadgwickSettings.accel_recovery_period)) ||
	    (MadgwickSettings.accel_recovery_period < 0.0f))
	{
		MadgwickSettings.accel_recovery_period = 5.0f;
	}
	if ((!madg_isFinite(MadgwickSettings.rest_gyro_threshold)) ||
	    (MadgwickSettings.rest_gyro_threshold <= 0.0f))
	{
		MadgwickSettings.rest_gyro_threshold = 3.0f * MADGWICK_PI / 180.0f;
	}
	if ((!madg_isFinite(MadgwickSettings.rest_accel_threshold)) ||
	    (MadgwickSettings.rest_accel_threshold <= 0.0f))
	{
		MadgwickSettings.rest_accel_threshold = 0.5f;
	}
	if ((!madg_isFinite(MadgwickSettings.rest_min_time)) || (MadgwickSettings.rest_min_time < 0.0f))
	{
		MadgwickSettings.rest_min_time = 1.5f;
	}
	if ((!madg_isFinite(MadgwickSettings.bias_tau)) || (MadgwickSettings.bias_tau <= 0.0f))
	{
		MadgwickSettings.bias_tau = 10.0f;
	}
	if ((!madg_isFinite(MadgwickSettings.bias_limit)) || (MadgwickSettings.bias_limit <= 0.0f))
	{
		MadgwickSettings.bias_limit = 0.10f;
	}
	if ((!madg_isFinite(MadgwickSettings.max_dt)) || (MadgwickSettings.max_dt <= 0.0f))
	{
		MadgwickSettings.max_dt = 0.02f;
	}
}

void MadgwickAHRSgetQuaternion(float quaternion[4])
{
	if (quaternion == NULL)
	{
		return;
	}

	quaternion[0] = madg_q0;
	quaternion[1] = madg_q1;
	quaternion[2] = madg_q2;
	quaternion[3] = madg_q3;
}

void MadgwickAHRSgetEuler(float *yaw, float *pitch, float *roll)
{
	const float q0 = madg_q0;
	const float q1 = madg_q1;
	const float q2 = madg_q2;
	const float q3 = madg_q3;
	const float sine = madg_clampf(-2.0f * (q1 * q3 - q0 * q2), -1.0f, 1.0f);

	if (yaw != NULL)
	{
		*yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
		              2.0f * (q0 * q0 + q1 * q1) - 1.0f);
	}
	if (pitch != NULL)
	{
		*pitch = atan2f(2.0f * (q0 * q1 + q2 * q3),
		                2.0f * (q0 * q0 + q3 * q3) - 1.0f);
	}
	if (roll != NULL)
	{
		*roll = asinf(sine);
	}
}

static uint8_t madg_validUpdate(float gx, float gy, float gz, float dt)
{
	/* dt 过大通常意味着任务堵塞；继续积分会造成不可控的姿态跳变。 */
	if ((!madg_vectorIsFinite(gx, gy, gz)) ||
	    (!madg_isFinite(dt)) ||
	    (dt <= 0.0f) ||
	    (dt > MadgwickSettings.max_dt))
	{
		MadgwickState.invalid_update_count++;
		return 0U;
	}
	return 1U;
}

static float madg_effectiveBeta(void)
{
	/* 启动时使用较强重力修正，随后线性过渡到运行期 beta。 */
	float target_beta = beta;

	if ((!madg_isFinite(target_beta)) || (target_beta < 0.0f))
	{
		target_beta = MADGWICK_DEFAULT_BETA;
	}

	if ((MadgwickSettings.startup_period > 0.0f) &&
	    (MadgwickState.elapsed_time < MadgwickSettings.startup_period))
	{
		const float ratio = MadgwickState.elapsed_time / MadgwickSettings.startup_period;
		return MadgwickSettings.startup_beta +
		       (target_beta - MadgwickSettings.startup_beta) * madg_clampf(ratio, 0.0f, 1.0f);
	}

	return target_beta;
}

/*
 * 对加速度低通滤波并计算可信度，同时检测静止、学习三轴残余陀螺仪零偏，
 * 最终返回归一化的加速度方向。返回 0 表示本次只能进行纯陀螺仪积分。
 */
static uint8_t madg_prepareImu(float *gx, float *gy, float *gz,
                               float *ax, float *ay, float *az,
                               float dt)
{
	float accel_norm;
	float reciprocal_norm;
	float gyro_norm;
	float magnitude_confidence;
	float direction_confidence;
	float confidence_target;
	float expected_x;
	float expected_y;
	float expected_z;
	float dot;
	float direction_error;
	float filter_alpha;
	float recovery_weight = 0.0f;
	uint8_t stationary_candidate;

	if (!madg_vectorIsFinite(*ax, *ay, *az))
	{
		MadgwickState.accel_confidence = 0.0f;
		MadgwickState.accel_ignored = 1U;
		MadgwickState.stationary = 0U;
		MadgwickState.rest_time = 0.0f;
		return 0U;
	}

	if (!MadgwickState.accel_initialized)
	{
		MadgwickState.accel_filtered[0] = *ax;
		MadgwickState.accel_filtered[1] = *ay;
		MadgwickState.accel_filtered[2] = *az;
		MadgwickState.accel_initialized = 1U;
	}
	else
	{
		filter_alpha = (MadgwickSettings.accel_lpf_tau > 0.0f)
		                 ? dt / (MadgwickSettings.accel_lpf_tau + dt)
		                 : 1.0f;
		filter_alpha = madg_clampf(filter_alpha, 0.0f, 1.0f);
		MadgwickState.accel_filtered[0] += filter_alpha * (*ax - MadgwickState.accel_filtered[0]);
		MadgwickState.accel_filtered[1] += filter_alpha * (*ay - MadgwickState.accel_filtered[1]);
		MadgwickState.accel_filtered[2] += filter_alpha * (*az - MadgwickState.accel_filtered[2]);
	}

	accel_norm = sqrtf(MadgwickState.accel_filtered[0] * MadgwickState.accel_filtered[0] +
	                   MadgwickState.accel_filtered[1] * MadgwickState.accel_filtered[1] +
	                   MadgwickState.accel_filtered[2] * MadgwickState.accel_filtered[2]);

	if ((!madg_isFinite(accel_norm)) || (accel_norm <= MADGWICK_NORM_EPSILON))
	{
		MadgwickState.accel_confidence = 0.0f;
		MadgwickState.accel_ignored = 1U;
		MadgwickState.stationary = 0U;
		MadgwickState.rest_time = 0.0f;
		return 0U;
	}

	MadgwickState.accel_magnitude_error = fabsf(accel_norm - MadgwickSettings.gravity);
	reciprocal_norm = 1.0f / accel_norm;
	*ax = MadgwickState.accel_filtered[0] * reciprocal_norm;
	*ay = MadgwickState.accel_filtered[1] * reciprocal_norm;
	*az = MadgwickState.accel_filtered[2] * reciprocal_norm;

	expected_x = 2.0f * (madg_q1 * madg_q3 - madg_q0 * madg_q2);
	expected_y = 2.0f * (madg_q0 * madg_q1 + madg_q2 * madg_q3);
	expected_z = 1.0f - 2.0f * (madg_q1 * madg_q1 + madg_q2 * madg_q2);
	dot = madg_clampf(*ax * expected_x + *ay * expected_y + *az * expected_z, -1.0f, 1.0f);

	/* sqrt(2*(1-dot)) 是低成本且在小角度下精确的方向误差近似。 */
	direction_error = sqrtf(madg_clampf(2.0f * (1.0f - dot), 0.0f, 4.0f));
	MadgwickState.accel_direction_error = direction_error;

	magnitude_confidence = madg_smoothstep(
		1.0f - MadgwickState.accel_magnitude_error / MadgwickSettings.accel_magnitude_rejection);
	direction_confidence = madg_smoothstep(
		1.0f - direction_error / MadgwickSettings.accel_direction_rejection);

	if (direction_confidence < 0.10f)
	{
		MadgwickState.rejection_time += dt;
	}
	else
	{
		MadgwickState.rejection_time = 0.0f;
	}

	/*
	 * 初始姿态较差时，方向判据可能长期拒绝重力修正。持续超时后仅逐步放宽
	 * 方向判据，模长判据仍然有效，避免把明显的线性加速度当作重力。
	 */
	if ((MadgwickSettings.accel_recovery_period > 0.0f) &&
	    (MadgwickState.rejection_time > MadgwickSettings.accel_recovery_period))
	{
		recovery_weight =
			(MadgwickState.rejection_time - MadgwickSettings.accel_recovery_period) /
			MadgwickSettings.accel_recovery_period;
		recovery_weight = madg_clampf(recovery_weight, 0.0f, 1.0f);
	}

	if (recovery_weight > direction_confidence)
	{
		direction_confidence = recovery_weight;
	}
	confidence_target = magnitude_confidence * direction_confidence;

	/* 风险出现时立即降低置信度，恢复时平滑上升，避免修正增益阶跃。 */
	if (confidence_target < MadgwickState.accel_confidence)
	{
		MadgwickState.accel_confidence = confidence_target;
	}
	else if (MadgwickSettings.accel_recovery_tau > 0.0f)
	{
		filter_alpha = dt / (MadgwickSettings.accel_recovery_tau + dt);
		MadgwickState.accel_confidence +=
			filter_alpha * (confidence_target - MadgwickState.accel_confidence);
	}
	else
	{
		MadgwickState.accel_confidence = confidence_target;
	}
	MadgwickState.accel_confidence =
		madg_clampf(MadgwickState.accel_confidence, 0.0f, 1.0f);
	MadgwickState.accel_ignored = (uint8_t)(MadgwickState.accel_confidence < 0.01f);
	if (MadgwickState.accel_ignored)
	{
		MadgwickState.accel_reject_count++;
	}

	*gx -= MadgwickState.gyro_bias[0];
	*gy -= MadgwickState.gyro_bias[1];
	*gz -= MadgwickState.gyro_bias[2];
	gyro_norm = sqrtf(*gx * *gx + *gy * *gy + *gz * *gz);

	stationary_candidate = (uint8_t)(
		(gyro_norm < MadgwickSettings.rest_gyro_threshold) &&
		(MadgwickState.accel_magnitude_error < MadgwickSettings.rest_accel_threshold));

	if (stationary_candidate)
	{
		MadgwickState.rest_time += dt;
	}
	else
	{
		MadgwickState.rest_time = 0.0f;
	}
	MadgwickState.stationary =
		(uint8_t)(MadgwickState.rest_time >= MadgwickSettings.rest_min_time);

	if (MadgwickState.stationary)
	{
		/* 仅在持续静止后缓慢学习零偏，并限制每轴估计范围。 */
		const float bias_alpha = dt / (MadgwickSettings.bias_tau + dt);
		uint8_t i;
		float raw_gyro[3];

		raw_gyro[0] = *gx + MadgwickState.gyro_bias[0];
		raw_gyro[1] = *gy + MadgwickState.gyro_bias[1];
		raw_gyro[2] = *gz + MadgwickState.gyro_bias[2];

		for (i = 0U; i < 3U; i++)
		{
			MadgwickState.gyro_bias[i] +=
				bias_alpha * (raw_gyro[i] - MadgwickState.gyro_bias[i]);
			MadgwickState.gyro_bias[i] =
				madg_clampf(MadgwickState.gyro_bias[i],
				            -MadgwickSettings.bias_limit,
				            MadgwickSettings.bias_limit);
		}

		*gx = raw_gyro[0] - MadgwickState.gyro_bias[0];
		*gy = raw_gyro[1] - MadgwickState.gyro_bias[1];
		*gz = raw_gyro[2] - MadgwickState.gyro_bias[2];
	}

	return 1U;
}

void MadgwickAHRSupdateIMUDt(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float dt)
{
	/* 六轴更新：陀螺仪负责预测，加速度梯度按可信度修正俯仰和横滚。 */
	float q0 = madg_q0;
	float q1 = madg_q1;
	float q2 = madg_q2;
	float q3 = madg_q3;
	float q_dot0;
	float q_dot1;
	float q_dot2;
	float q_dot3;
	uint8_t accel_valid;

	if (!madg_validUpdate(gx, gy, gz, dt))
	{
		return;
	}

	accel_valid = madg_prepareImu(&gx, &gy, &gz, &ax, &ay, &az, dt);

	q_dot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	q_dot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
	q_dot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
	q_dot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

	if (accel_valid && (!MadgwickState.accel_ignored))
	{
		const float twice_q0 = 2.0f * q0;
		const float twice_q1 = 2.0f * q1;
		const float twice_q2 = 2.0f * q2;
		const float twice_q3 = 2.0f * q3;
		const float four_q0 = 4.0f * q0;
		const float four_q1 = 4.0f * q1;
		const float four_q2 = 4.0f * q2;
		const float eight_q1 = 8.0f * q1;
		const float eight_q2 = 8.0f * q2;
		const float q0q0 = q0 * q0;
		const float q1q1 = q1 * q1;
		const float q2q2 = q2 * q2;
		const float q3q3 = q3 * q3;
		float s0;
		float s1;
		float s2;
		float s3;
		float reciprocal_step_norm;

		s0 = four_q0 * q2q2 + twice_q2 * ax + four_q0 * q1q1 - twice_q1 * ay;
		s1 = four_q1 * q3q3 - twice_q3 * ax + 4.0f * q0q0 * q1 -
		     twice_q0 * ay - four_q1 + eight_q1 * q1q1 +
		     eight_q1 * q2q2 + four_q1 * az;
		s2 = 4.0f * q0q0 * q2 + twice_q0 * ax + four_q2 * q3q3 -
		     twice_q3 * ay - four_q2 + eight_q2 * q1q1 +
		     eight_q2 * q2q2 + four_q2 * az;
		s3 = 4.0f * q1q1 * q3 - twice_q1 * ax +
		     4.0f * q2q2 * q3 - twice_q2 * ay;

		reciprocal_step_norm = madg_invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		if (reciprocal_step_norm > 0.0f)
		{
			const float feedback_gain =
				madg_effectiveBeta() * MadgwickState.accel_confidence;
			s0 *= reciprocal_step_norm;
			s1 *= reciprocal_step_norm;
			s2 *= reciprocal_step_norm;
			s3 *= reciprocal_step_norm;
			q_dot0 -= feedback_gain * s0;
			q_dot1 -= feedback_gain * s1;
			q_dot2 -= feedback_gain * s2;
			q_dot3 -= feedback_gain * s3;
		}
	}

	/* 积分后必须重新归一化；失败时保留上一份有效四元数。 */
	q0 += q_dot0 * dt;
	q1 += q_dot1 * dt;
	q2 += q_dot2 * dt;
	q3 += q_dot3 * dt;

	if (!madg_normaliseQuaternion(q0, q1, q2, q3))
	{
		MadgwickState.invalid_update_count++;
	}
	else
	{
		MadgwickState.elapsed_time += dt;
		MadgwickState.update_count++;
	}
}

void MadgwickAHRSupdateDt(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt)
{
	/* 九轴更新：磁场无效时自动退化为六轴，不中断姿态输出。 */
	float q0;
	float q1;
	float q2;
	float q3;
	float q_dot0;
	float q_dot1;
	float q_dot2;
	float q_dot3;
	float reciprocal_norm;
	float mag_norm_sq;
	uint8_t accel_valid;

	mag_norm_sq = mx * mx + my * my + mz * mz;
	if ((!madg_vectorIsFinite(mx, my, mz)) ||
	    (!madg_isFinite(mag_norm_sq)) ||
	    (mag_norm_sq <= MADGWICK_NORM_EPSILON))
	{
		MadgwickAHRSupdateIMUDt(gx, gy, gz, ax, ay, az, dt);
		return;
	}

	if (!madg_validUpdate(gx, gy, gz, dt))
	{
		return;
	}

	q0 = madg_q0;
	q1 = madg_q1;
	q2 = madg_q2;
	q3 = madg_q3;
	accel_valid = madg_prepareImu(&gx, &gy, &gz, &ax, &ay, &az, dt);

	q_dot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	q_dot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
	q_dot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
	q_dot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

	if (accel_valid && (!MadgwickState.accel_ignored))
	{
		/* 同时利用重力和归一化磁场构造 Madgwick 梯度下降修正项。 */
		const float twice_q0 = 2.0f * q0;
		const float twice_q1 = 2.0f * q1;
		const float twice_q2 = 2.0f * q2;
		const float twice_q3 = 2.0f * q3;
		const float q0q0 = q0 * q0;
		const float q0q1 = q0 * q1;
		const float q0q2 = q0 * q2;
		const float q0q3 = q0 * q3;
		const float q1q1 = q1 * q1;
		const float q1q2 = q1 * q2;
		const float q1q3 = q1 * q3;
		const float q2q2 = q2 * q2;
		const float q2q3 = q2 * q3;
		const float q3q3 = q3 * q3;
		float twice_q0mx;
		float twice_q0my;
		float twice_q0mz;
		float twice_q1mx;
		float twice_bx;
		float twice_bz;
		float four_bx;
		float four_bz;
		float hx;
		float hy;
		float s0;
		float s1;
		float s2;
		float s3;
		float reciprocal_step_norm;

		reciprocal_norm = madg_invSqrt(mag_norm_sq);
		mx *= reciprocal_norm;
		my *= reciprocal_norm;
		mz *= reciprocal_norm;

		twice_q0mx = 2.0f * q0 * mx;
		twice_q0my = 2.0f * q0 * my;
		twice_q0mz = 2.0f * q0 * mz;
		twice_q1mx = 2.0f * q1 * mx;

		hx = mx * q0q0 - twice_q0my * q3 + twice_q0mz * q2 +
		     mx * q1q1 + twice_q1 * my * q2 + twice_q1 * mz * q3 -
		     mx * q2q2 - mx * q3q3;
		hy = twice_q0mx * q3 + my * q0q0 - twice_q0mz * q1 +
		     twice_q1mx * q2 - my * q1q1 + my * q2q2 +
		     twice_q2 * mz * q3 - my * q3q3;
		twice_bx = sqrtf(hx * hx + hy * hy);
		twice_bz = -twice_q0mx * q2 + twice_q0my * q1 + mz * q0q0 +
		           twice_q1mx * q3 - mz * q1q1 + twice_q2 * my * q3 -
		           mz * q2q2 + mz * q3q3;
		four_bx = 2.0f * twice_bx;
		four_bz = 2.0f * twice_bz;

		s0 = -twice_q2 * (2.0f * q1q3 - 2.0f * q0q2 - ax) +
		     twice_q1 * (2.0f * q0q1 + 2.0f * q2q3 - ay) -
		     twice_bz * q2 * (twice_bx * (0.5f - q2q2 - q3q3) +
		                      twice_bz * (q1q3 - q0q2) - mx) +
		     (-twice_bx * q3 + twice_bz * q1) *
		         (twice_bx * (q1q2 - q0q3) + twice_bz * (q0q1 + q2q3) - my) +
		     twice_bx * q2 * (twice_bx * (q0q2 + q1q3) +
		                      twice_bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = twice_q3 * (2.0f * q1q3 - 2.0f * q0q2 - ax) +
		     twice_q0 * (2.0f * q0q1 + 2.0f * q2q3 - ay) -
		     4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
		     twice_bz * q3 * (twice_bx * (0.5f - q2q2 - q3q3) +
		                      twice_bz * (q1q3 - q0q2) - mx) +
		     (twice_bx * q2 + twice_bz * q0) *
		         (twice_bx * (q1q2 - q0q3) + twice_bz * (q0q1 + q2q3) - my) +
		     (twice_bx * q3 - four_bz * q1) *
		         (twice_bx * (q0q2 + q1q3) +
		          twice_bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -twice_q0 * (2.0f * q1q3 - 2.0f * q0q2 - ax) +
		     twice_q3 * (2.0f * q0q1 + 2.0f * q2q3 - ay) -
		     4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
		     (-four_bx * q2 - twice_bz * q0) *
		         (twice_bx * (0.5f - q2q2 - q3q3) +
		          twice_bz * (q1q3 - q0q2) - mx) +
		     (twice_bx * q1 + twice_bz * q3) *
		         (twice_bx * (q1q2 - q0q3) + twice_bz * (q0q1 + q2q3) - my) +
		     (twice_bx * q0 - four_bz * q2) *
		         (twice_bx * (q0q2 + q1q3) +
		          twice_bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = twice_q1 * (2.0f * q1q3 - 2.0f * q0q2 - ax) +
		     twice_q2 * (2.0f * q0q1 + 2.0f * q2q3 - ay) +
		     (-four_bx * q3 + twice_bz * q1) *
		         (twice_bx * (0.5f - q2q2 - q3q3) +
		          twice_bz * (q1q3 - q0q2) - mx) +
		     (-twice_bx * q0 + twice_bz * q2) *
		         (twice_bx * (q1q2 - q0q3) + twice_bz * (q0q1 + q2q3) - my) +
		     twice_bx * q1 * (twice_bx * (q0q2 + q1q3) +
		                      twice_bz * (0.5f - q1q1 - q2q2) - mz);

		reciprocal_step_norm = madg_invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		if (reciprocal_step_norm > 0.0f)
		{
			const float feedback_gain =
				madg_effectiveBeta() * MadgwickState.accel_confidence;
			s0 *= reciprocal_step_norm;
			s1 *= reciprocal_step_norm;
			s2 *= reciprocal_step_norm;
			s3 *= reciprocal_step_norm;
			q_dot0 -= feedback_gain * s0;
			q_dot1 -= feedback_gain * s1;
			q_dot2 -= feedback_gain * s2;
			q_dot3 -= feedback_gain * s3;
		}
	}

	q0 += q_dot0 * dt;
	q1 += q_dot1 * dt;
	q2 += q_dot2 * dt;
	q3 += q_dot3 * dt;

	if (!madg_normaliseQuaternion(q0, q1, q2, q3))
	{
		MadgwickState.invalid_update_count++;
	}
	else
	{
		MadgwickState.elapsed_time += dt;
		MadgwickState.update_count++;
	}
}

void MadgwickAHRSupdate(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz)
{
	/* 旧接口没有 dt 参数，只能维持历史约定的 1 ms 采样周期。 */
	MadgwickAHRSupdateDt(gx, gy, gz, ax, ay, az, mx, my, mz,
	                    MADGWICK_DEFAULT_SAMPLE_PERIOD);
}

void MadgwickAHRSupdateIMU(float gx, float gy, float gz,
                           float ax, float ay, float az)
{
	/* 旧接口没有 dt 参数，只能维持历史约定的 1 ms 采样周期。 */
	MadgwickAHRSupdateIMUDt(gx, gy, gz, ax, ay, az,
	                       MADGWICK_DEFAULT_SAMPLE_PERIOD);
}
