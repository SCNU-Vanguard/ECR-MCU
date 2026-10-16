#ifndef AHRS_H__
#define AHRS_H__
/* 姿态解算统一接口：上层只依赖这里。用哪个算法在 cdd/imu/ins/ins.c 的 INS_ALGORITHM。
 * 每个算法由 CONFIG_COMPONENT_FILTER_<NAME> 决定是否编进固件：头文件、状态、调用都只在对应宏打开时存在。
 * 选了没编进来的算法时 ahrs_start 返回 ERR_NODEV。
 * 坐标约定：四元数 [w, x, y, z]，Hamilton，机体系 -> 导航系；静止时比力沿 +Z；欧拉角按 Z-Y-X（yaw-pitch-roll）。 */
#include <stdbool.h>
#include <stdint.h>
#ifdef CONFIG_COMPONENT_FILTER_EKF
#include "component/filter/ekf/quaternionekf.h"
#endif
#ifdef CONFIG_COMPONENT_FILTER_MADGWICK
#include "component/filter/madgwick/madgwickahrs.h"
#endif
#ifdef CONFIG_COMPONENT_FILTER_MAHONY
#include "component/filter/mahony/mahony_filter.h"
#endif
#ifdef CONFIG_COMPONENT_FILTER_MEKF
#include "component/filter/mekf/mekf_fliter.h"
#endif
#ifdef CONFIG_COMPONENT_FILTER_VQF
#include "component/filter/vqf/vqf_filter.h"
#endif

typedef enum
{
    AHRS_ALGORITHM_VQF = 0,
    AHRS_ALGORITHM_MEKF,
    AHRS_ALGORITHM_MADGWICK,
    AHRS_ALGORITHM_MAHONY,
    AHRS_ALGORITHM_EKF,
    AHRS_ALGORITHM_COUNT
} ahrs_algorithm_e;

typedef struct
{
    float gyro_rad_s[3];         /* 已标定的角速度 */
    float accel_m_s2[3];         /* 已标定的比力 */
    float dt_s;                  /* 与上一个样本的真实间隔，合法范围 [1e-5, 0.02] */
    bool  gyro_clipped;          /* 陀螺饱和 */
    bool  accel_clipped;         /* 加速度计饱和 */
    bool  stationary;            /* 外部判断的静止（只有 MEKF 使用） */
} ahrs_sample_t;

typedef struct
{
    ahrs_algorithm_e algorithm;
    bool  initialized;                       /* 算法状态是否已经用首帧有效加速度建立 */
    float nominal_dt_s;                      /* 名义采样周期，只用于需要固定周期初始化的算法 */
    float quaternion[4];
    float roll_rad;
    float pitch_rad;
    float yaw_rad;                           /* (-pi, pi] */
    float yaw_total_rad;                     /* 连续累计 yaw，跨 ±pi 不跳变 */
    uint32_t update_count;                   /* 成功推进的次数 */
    uint32_t reject_count;                   /* 样本被拒绝或算法数值失败的次数 */
    /* Madgwick / EKF 的状态在各自模块的全局变量里，不放在这里 */
#if defined(CONFIG_COMPONENT_FILTER_VQF) || defined(CONFIG_COMPONENT_FILTER_MEKF) || defined(CONFIG_COMPONENT_FILTER_MAHONY)
    union
    {
#ifdef CONFIG_COMPONENT_FILTER_VQF
        VQF_State vqf;
#endif
#ifdef CONFIG_COMPONENT_FILTER_MEKF
        MEKF_State mekf;
#endif
#ifdef CONFIG_COMPONENT_FILTER_MAHONY
        mahony_filter_t mahony;
#endif
    } state;
#endif
} ahrs_t;

/* 选算法并复位。真正的初始化推迟到第一帧有效加速度（要用它算初始姿态），在那之前 ahrs_update 返回 ERR_BUSY。
 * EKF 在初始化时用 pvPortMalloc 分配矩阵且没有释放接口，所以每次启动只能 ahrs_start 一次 EKF */
int32_t ahrs_start(ahrs_t *ahrs, ahrs_algorithm_e algorithm, float nominal_dt_s);
/* 喂入一个样本：ERR_OK 已推进；ERR_BUSY 还在等首帧有效加速度；ERR_INVAL 样本被拒绝；ERR_FAIL 算法数值失败 */
int32_t ahrs_update(ahrs_t *ahrs, const ahrs_sample_t *sample);
/* 数据流中断（丢帧、重启采集）后调用，让算法不要跨缺口累计 */
void ahrs_break_continuity(ahrs_t *ahrs);

bool ahrs_algorithm_enabled(ahrs_algorithm_e algorithm);
const char *ahrs_algorithm_name(ahrs_algorithm_e algorithm);
#endif
