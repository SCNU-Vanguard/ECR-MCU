#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "component/filter/ahrs.h"
#include "component/common/err.h"


#define AHRS_PI                     3.14159265358979f
#define AHRS_TWO_PI                 6.28318530717959f
#define AHRS_DT_MIN_S               1.0e-5f
#define AHRS_DT_MAX_S               0.02f

/* 首帧加速度的模长在这个范围内才认为"基本只受重力"，可以用来算初始姿态 */
#define AHRS_INIT_ACCEL_MIN_M_S2    7.0f
#define AHRS_INIT_ACCEL_MAX_M_S2    12.6f

/* Mahony 增益：比例项越大越信加速度计，积分项用来消陀螺零偏。起步值，未针对本机调过 */
#define AHRS_MAHONY_KP              1.0f
#define AHRS_MAHONY_KI              0.002f

/* EKF 参数：过程噪声(四元数)、过程噪声(零偏)、测量噪声、渐消因子、加速度低通。沿用原仓库 INS 中 EKF 的数值 */
#define AHRS_KALMAN_PROCESS_NOISE_QUATERNION    10.5f
#define AHRS_KALMAN_PROCESS_NOISE_BIAS          0.00015f
#define AHRS_KALMAN_MEASURE_NOISE               1000000.0f
#define AHRS_KALMAN_FADING_FACTOR               0.9998f
#define AHRS_KALMAN_ACCEL_LPF                   0.0005f

static const char *const algorithm_names[AHRS_ALGORITHM_COUNT] =
{
    [AHRS_ALGORITHM_VQF]      = "vqf",
    [AHRS_ALGORITHM_MEKF]     = "mekf",
    [AHRS_ALGORITHM_MADGWICK] = "madgwick",
    [AHRS_ALGORITHM_MAHONY]   = "mahony",
    [AHRS_ALGORITHM_EKF]      = "ekf",
};

bool ahrs_algorithm_enabled(ahrs_algorithm_e algorithm)
{
    switch (algorithm)
    {
#ifdef CONFIG_COMPONENT_FILTER_VQF
    case AHRS_ALGORITHM_VQF:
        return true;
#endif
#ifdef CONFIG_COMPONENT_FILTER_MEKF
    case AHRS_ALGORITHM_MEKF:
        return true;
#endif
#ifdef CONFIG_COMPONENT_FILTER_MADGWICK
    case AHRS_ALGORITHM_MADGWICK:
        return true;
#endif
#ifdef CONFIG_COMPONENT_FILTER_MAHONY
    case AHRS_ALGORITHM_MAHONY:
        return true;
#endif
#ifdef CONFIG_COMPONENT_FILTER_EKF
    case AHRS_ALGORITHM_EKF:
        return true;
#endif
    default:
        return false;
    }
}

const char *ahrs_algorithm_name(ahrs_algorithm_e algorithm)
{
    if ((unsigned)algorithm >= AHRS_ALGORITHM_COUNT)
    {
        return "?";
    }
    return algorithm_names[algorithm];
}

int32_t ahrs_start(ahrs_t *ahrs, ahrs_algorithm_e algorithm, float nominal_dt_s)
{
    if (ahrs == NULL || (unsigned)algorithm >= AHRS_ALGORITHM_COUNT || !(nominal_dt_s >= AHRS_DT_MIN_S && nominal_dt_s <= AHRS_DT_MAX_S))
    {
        return ERR_INVAL;
    }
    if (!ahrs_algorithm_enabled(algorithm))
    {
        return ERR_NODEV;
    }
    memset(ahrs, 0, sizeof *ahrs);
    ahrs->algorithm = algorithm;
    ahrs->nominal_dt_s = nominal_dt_s;
    ahrs->quaternion[0] = 1.0f;
    return ERR_OK;
}

/* 用加速度方向算初始姿态：roll/pitch 由重力方向得到，yaw 取 0 */
static bool initial_quaternion_from_accel(const float accel[3], float quaternion[4])
{
    float norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

    if (!(norm >= AHRS_INIT_ACCEL_MIN_M_S2 && norm <= AHRS_INIT_ACCEL_MAX_M_S2))
    {
        return false;
    }
    float roll = atan2f(accel[1], accel[2]);
    float pitch = atan2f(-accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2]));
    float half_roll_cos = cosf(roll * 0.5f);
    float half_roll_sin = sinf(roll * 0.5f);
    float half_pitch_cos = cosf(pitch * 0.5f);
    float half_pitch_sin = sinf(pitch * 0.5f);

    quaternion[0] = half_roll_cos * half_pitch_cos;
    quaternion[1] = half_roll_sin * half_pitch_cos;
    quaternion[2] = half_roll_cos * half_pitch_sin;
    quaternion[3] = -half_roll_sin * half_pitch_sin;
    return true;
}

static int32_t algorithm_initialize(ahrs_t *ahrs, const float initial_quaternion[4])
{
    switch (ahrs->algorithm)
    {
#ifdef CONFIG_COMPONENT_FILTER_VQF
    case AHRS_ALGORITHM_VQF:
        return VQF_Init(&ahrs->state.vqf, initial_quaternion, ahrs->nominal_dt_s) ? ERR_OK : ERR_FAIL;
#endif
#ifdef CONFIG_COMPONENT_FILTER_MEKF
    case AHRS_ALGORITHM_MEKF:
        return MEKF_Init(&ahrs->state.mekf, initial_quaternion, NULL, NULL) ? ERR_OK : ERR_FAIL;
#endif
#ifdef CONFIG_COMPONENT_FILTER_MADGWICK
    case AHRS_ALGORITHM_MADGWICK:
        MadgwickAHRSinit(initial_quaternion);
        return ERR_OK;
#endif
#ifdef CONFIG_COMPONENT_FILTER_MAHONY
    case AHRS_ALGORITHM_MAHONY:
        Mahony_Filter_Init(&ahrs->state.mahony, AHRS_MAHONY_KP, AHRS_MAHONY_KI, ahrs->nominal_dt_s);
        ahrs->state.mahony.q0 = initial_quaternion[0];
        ahrs->state.mahony.q1 = initial_quaternion[1];
        ahrs->state.mahony.q2 = initial_quaternion[2];
        ahrs->state.mahony.q3 = initial_quaternion[3];
        Mahony_Filter_RotationMatrix_Update(&ahrs->state.mahony);
        return ERR_OK;
#endif
#ifdef CONFIG_COMPONENT_FILTER_EKF
    case AHRS_ALGORITHM_EKF:
    {
        float quaternion_copy[4] = { initial_quaternion[0], initial_quaternion[1], initial_quaternion[2], initial_quaternion[3] };

        IMU_QuaternionEKF_Init(quaternion_copy, AHRS_KALMAN_PROCESS_NOISE_QUATERNION, AHRS_KALMAN_PROCESS_NOISE_BIAS,
                               AHRS_KALMAN_MEASURE_NOISE, AHRS_KALMAN_FADING_FACTOR, AHRS_KALMAN_ACCEL_LPF);
        return ERR_OK;
    }
#endif
    default:
        return ERR_NODEV;
    }
}

static void publish_quaternion(ahrs_t *ahrs, const float quaternion[4])
{
    float norm = sqrtf(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
                       quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);

    if (!(norm > 0.5f && norm < 2.0f))
    {
        return;
    }
    for (int component_index = 0; component_index < 4; component_index++)
    {
        ahrs->quaternion[component_index] = quaternion[component_index] / norm;
    }

    float w = ahrs->quaternion[0];
    float x = ahrs->quaternion[1];
    float y = ahrs->quaternion[2];
    float z = ahrs->quaternion[3];
    float sin_pitch = 2.0f * (w * y - z * x);

    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }
    float previous_yaw = ahrs->yaw_rad;

    ahrs->roll_rad = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    ahrs->pitch_rad = asinf(sin_pitch);
    ahrs->yaw_rad = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

    float yaw_step = ahrs->yaw_rad - previous_yaw;

    if (ahrs->update_count == 0u)
    {
        ahrs->yaw_total_rad = ahrs->yaw_rad;
        return;
    }
    if (yaw_step > AHRS_PI)
    {
        yaw_step -= AHRS_TWO_PI;
    }
    else if (yaw_step < -AHRS_PI)
    {
        yaw_step += AHRS_TWO_PI;
    }
    ahrs->yaw_total_rad += yaw_step;
}

int32_t ahrs_update(ahrs_t *ahrs, const ahrs_sample_t *sample)
{
    if (ahrs == NULL || sample == NULL)
    {
        return ERR_INVAL;
    }
    if (!(sample->dt_s >= AHRS_DT_MIN_S && sample->dt_s <= AHRS_DT_MAX_S))
    {
        ahrs->reject_count++;
        return ERR_INVAL;
    }

    if (!ahrs->initialized)
    {
        float initial_quaternion[4];

        if (!initial_quaternion_from_accel(sample->accel_m_s2, initial_quaternion))
        {
            return ERR_BUSY;
        }
        int32_t init_result = algorithm_initialize(ahrs, initial_quaternion);

        if (init_result != ERR_OK)
        {
            return init_result;
        }
        ahrs->initialized = true;
        publish_quaternion(ahrs, initial_quaternion);
        return ERR_BUSY;                            /* 这一帧只用来建立初值，下一帧才开始推进 */
    }

    const float *gyro = sample->gyro_rad_s;
    const float *accel = sample->accel_m_s2;
    float dt = sample->dt_s;
    float quaternion[4];
    bool predicted = false;

    switch (ahrs->algorithm)
    {
#ifdef CONFIG_COMPONENT_FILTER_VQF
    case AHRS_ALGORITHM_VQF:
    {
        uint8_t flags = (uint8_t)((sample->gyro_clipped ? VQF_GYRO_CLIPPED : 0u) | (sample->accel_clipped ? VQF_ACCEL_CLIPPED : 0u));
        uint8_t status = VQF_Update(&ahrs->state.vqf, gyro, accel, dt, flags);

        predicted = (status & VQF_PREDICTED) != 0u;
        memcpy(quaternion, ahrs->state.vqf.filter.q, sizeof quaternion);
        break;
    }
#endif
#ifdef CONFIG_COMPONENT_FILTER_MEKF
    case AHRS_ALGORITHM_MEKF:
    {
        uint8_t flags = (uint8_t)((sample->gyro_clipped ? MEKF_GYRO_CLIPPED : 0u) | (sample->accel_clipped ? MEKF_ACCEL_CLIPPED : 0u) |
                                  (sample->stationary ? MEKF_STATIONARY : 0u));
        uint8_t status = MEKF_Update(&ahrs->state.mekf, gyro, accel, dt, flags);

        predicted = (status & MEKF_PREDICTED) != 0u;
        memcpy(quaternion, ahrs->state.mekf.q, sizeof quaternion);
        break;
    }
#endif
#ifdef CONFIG_COMPONENT_FILTER_MADGWICK
    case AHRS_ALGORITHM_MADGWICK:
        MadgwickAHRSupdateIMUDt(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], dt);
        MadgwickAHRSgetQuaternion(quaternion);
        predicted = true;
        break;
#endif
#ifdef CONFIG_COMPONENT_FILTER_MAHONY
    case AHRS_ALGORITHM_MAHONY:
    {
        axis_3f_t gyro_axes = { gyro[0], gyro[1], gyro[2] };
        axis_3f_t accel_axes = { accel[0], accel[1], accel[2] };

        ahrs->state.mahony.dt = dt;
        Mahony_Filter_Input(&ahrs->state.mahony, gyro_axes, accel_axes);
        Mahony_Filter_Update(&ahrs->state.mahony);
        quaternion[0] = ahrs->state.mahony.q0;
        quaternion[1] = ahrs->state.mahony.q1;
        quaternion[2] = ahrs->state.mahony.q2;
        quaternion[3] = ahrs->state.mahony.q3;
        predicted = true;
        break;
    }
#endif
#ifdef CONFIG_COMPONENT_FILTER_EKF
    case AHRS_ALGORITHM_EKF:
        IMU_QuaternionEKF_Update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], dt);
        memcpy(quaternion, QEKF_INS.q, sizeof quaternion);
        predicted = true;
        break;
#endif
    default:
        return ERR_NODEV;
    }

    if (!predicted)
    {
        ahrs->reject_count++;
        return ERR_INVAL;
    }
    for (int component_index = 0; component_index < 4; component_index++)
    {
        if (!isfinite(quaternion[component_index]))
        {
            ahrs->reject_count++;
            return ERR_FAIL;
        }
    }
    publish_quaternion(ahrs, quaternion);
    ahrs->update_count++;
    return ERR_OK;
}

void ahrs_break_continuity(ahrs_t *ahrs)
{
    if (ahrs == NULL || !ahrs->initialized)
    {
        return;
    }
#ifdef CONFIG_COMPONENT_FILTER_VQF
    if (ahrs->algorithm == AHRS_ALGORITHM_VQF)
    {
        VQF_BreakContinuity(&ahrs->state.vqf);
    }
#endif
}
