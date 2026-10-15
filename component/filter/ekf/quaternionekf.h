/*
 * @Author: GUATAI 2508588132@qq.com
 * @Date: 2026-02-11 00:00:43
 * @LastEditors: GUATAI 2508588132@qq.com
 * @LastEditTime: 2026-03-10 17:51:55
 * @FilePath: \2026_VANGUARD_Frame\algorithm\filter\EKF\quaternionekf.h
 * @Description: 
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
 */
/**
 ******************************************************************************
 * @file    quaternionekf.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H

#include "component/filter/kalman/kalman_filter.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // 四元数估计值
    float GyroBias[3]; // 陀螺仪零偏估计值

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计量测噪声

    float dt; // 姿态更新周期，秒
    mat ChiSquare;
    float ChiSquare_Data[1];      // 卡方检验检测函数
    float ChiSquareTestThreshold; // 卡方检验阈值
    float lambda;                 // 渐消因子

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;

extern float chiSquare;
extern float ChiSquareTestThreshold;
/** 初始化全局 QEKF_INS；init_quaternion 为非空单位四元数 [w,x,y,z]。
 * 噪声参数须有限且合理，0 < lambda <= 1，lpf 为非负加速度低通参数。
 * 内部通过 Kalman_Filter_Init 分配内存，不检查堆耗尽，不可在周期循环重复初始化。 */
void IMU_QuaternionEKF_Init(float* init_quaternion,float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf);
/** 单任务更新全局状态：角速度 rad/s，比力 m/s²，0 < dt < 1 秒。
 * 调用者过滤非有限值、零模长与饱和样本；未初始化或 dt 越界时直接返回。
 * 欧拉角输出为 rad，历史命名 Pitch=X、Roll=Y；业务优先通过 ins.h 使用统一接口。 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif
