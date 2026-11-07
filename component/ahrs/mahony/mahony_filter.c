/**
******************************************************************************
 * @file    mahony_filter.c
 * @brief 实例式 Mahony IMU 姿态融合与欧拉角输出。
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "component/ahrs/mahony/mahony_filter.h"

/*-----------------------MahonyAHRS滤波器----------------------*/
/*-----------------------------------------------------------*/
mahony_filter_t mahony_filter;

/*计算旋转矩阵*/	//大地坐标系 R 转换到机体坐标系 b 的坐标转换矩阵
void Mahony_Filter_RotationMatrix_Update(mahony_filter_t *mahony_filter)
{
  float q1q1 = mahony_filter->q1 * mahony_filter->q1;
  float q2q2 = mahony_filter->q2 * mahony_filter->q2;
  float q3q3 = mahony_filter->q3 * mahony_filter->q3;

  float q0q1 = mahony_filter->q0 * mahony_filter->q1;
  float q0q2 = mahony_filter->q0 * mahony_filter->q2;
  float q0q3 = mahony_filter->q0 * mahony_filter->q3;
  float q1q2 = mahony_filter->q1 * mahony_filter->q2;
  float q1q3 = mahony_filter->q1 * mahony_filter->q3;
  float q2q3 = mahony_filter->q2 * mahony_filter->q3;

  mahony_filter->rotation_matrix[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
  mahony_filter->rotation_matrix[0][1] = 2.0f * (q1q2 - q0q3);
  mahony_filter->rotation_matrix[0][2] = 2.0f * (q1q3 + q0q2);

  mahony_filter->rotation_matrix[1][0] = 2.0f * (q1q2 + q0q3);
  mahony_filter->rotation_matrix[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
  mahony_filter->rotation_matrix[1][2] = 2.0f * (q2q3 - q0q1);

  mahony_filter->rotation_matrix[2][0] = 2.0f * (q1q3 - q0q2);
  mahony_filter->rotation_matrix[2][1] = 2.0f * (q2q3 + q0q1);
  mahony_filter->rotation_matrix[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

// 将陀螺仪和加速度计的输入值赋予滤波器结构体
void Mahony_Filter_Input(mahony_filter_t *mahony_filter, axis_3f_t gyro, axis_3f_t acc)
{
  mahony_filter->gyro = gyro;
  mahony_filter->acc = acc;
}

/* 明确静止时直接观测 gyro + integral = 0，补足重力无法观测的航向零偏。
 * 每个有效样本都参与一阶低通，避免抽取单帧造成混叠；时间常数 10 s。 */
void Mahony_Filter_ObserveZeroRate(mahony_filter_t *f, axis_3f_t gyro, bool stationary, float dt)
{
  f->zero_rate_active = false;
  if (!stationary || !isfinite(dt) || dt <= 0.0f || dt > 0.02f ||
      !isfinite(gyro.x) || !isfinite(gyro.y) || !isfinite(gyro.z))
  {
    f->stationary_elapsed = 0.0f;
    return;
  }
  f->stationary_elapsed = fminf(f->stationary_elapsed + dt, 0.5f);
  if (f->stationary_elapsed < 0.5f) return;
  const float alpha = dt / (10.0f + dt);
  f->ex_intergral += alpha * (-gyro.x - f->ex_intergral);
  f->ey_intergral += alpha * (-gyro.y - f->ey_intergral);
  f->ez_intergral += alpha * (-gyro.z - f->ez_intergral);
  f->zero_rate_active = true;
  f->zero_rate_count++;
}

// 使用Mahony算法更新滤波器状态
void Mahony_Filter_Update(mahony_filter_t *mahony_filter)
{
  float normalise;
  float ex, ey, ez;

  /* 无可信加速度时只传播，保留已学习的积分反馈，不更新积分项。 */
  ex = ey = ez = 0.0f;
  normalise = sqrtf(mahony_filter->acc.x * mahony_filter->acc.x +
                    mahony_filter->acc.y * mahony_filter->acc.y +
                    mahony_filter->acc.z * mahony_filter->acc.z);
  if (isfinite(normalise) && normalise > 1.0e-6f)
  {
    mahony_filter->acc.x /= normalise;
    mahony_filter->acc.y /= normalise;
    mahony_filter->acc.z /= normalise;

    ex = mahony_filter->acc.y * mahony_filter->rotation_matrix[2][2] -
         mahony_filter->acc.z * mahony_filter->rotation_matrix[2][1];
    ey = mahony_filter->acc.z * mahony_filter->rotation_matrix[2][0] -
         mahony_filter->acc.x * mahony_filter->rotation_matrix[2][2];
    ez = mahony_filter->acc.x * mahony_filter->rotation_matrix[2][1] -
         mahony_filter->acc.y * mahony_filter->rotation_matrix[2][0];

    if (!mahony_filter->zero_rate_active)
    {
      mahony_filter->ex_intergral += mahony_filter->Ki * ex * mahony_filter->dt;
      mahony_filter->ey_intergral += mahony_filter->Ki * ey * mahony_filter->dt;
      mahony_filter->ez_intergral += mahony_filter->Ki * ez * mahony_filter->dt;
    }
  }

  /*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
  mahony_filter->gyro.x += mahony_filter->Kp * ex + mahony_filter->ex_intergral;
  mahony_filter->gyro.y += mahony_filter->Kp * ey + mahony_filter->ey_intergral;
  mahony_filter->gyro.z += mahony_filter->Kp * ez + mahony_filter->ez_intergral;

  /* 一阶近似算法，四元数运动学方程的离散化形式和积分 */
  float q0Last = mahony_filter->q0;
  float q1Last = mahony_filter->q1;
  float q2Last = mahony_filter->q2;
  float q3Last = mahony_filter->q3;
  float halfT = mahony_filter->dt * 0.5f;
  mahony_filter->q0 += (-q1Last * mahony_filter->gyro.x - q2Last * mahony_filter->gyro.y - q3Last * mahony_filter->gyro.z) * halfT;
  mahony_filter->q1 += (q0Last * mahony_filter->gyro.x + q2Last * mahony_filter->gyro.z - q3Last * mahony_filter->gyro.y) * halfT;
  mahony_filter->q2 += (q0Last * mahony_filter->gyro.y - q1Last * mahony_filter->gyro.z + q3Last * mahony_filter->gyro.x) * halfT;
  mahony_filter->q3 += (q0Last * mahony_filter->gyro.z + q1Last * mahony_filter->gyro.y - q2Last * mahony_filter->gyro.x) * halfT;

  /*单位化四元数*/
  normalise = sqrtf(mahony_filter->q0 * mahony_filter->q0 + mahony_filter->q1 * mahony_filter->q1 + mahony_filter->q2 * mahony_filter->q2 + mahony_filter->q3 * mahony_filter->q3);

  mahony_filter->q0 = mahony_filter->q0 / normalise;
  mahony_filter->q1 = mahony_filter->q1 / normalise;
  mahony_filter->q2 = mahony_filter->q2 / normalise;
  mahony_filter->q3 = mahony_filter->q3 / normalise;

  /*计算旋转矩阵*/
  mahony_filter->Mahony_Filter_RotationMatrix_Update(mahony_filter);
}

// 从旋转矩阵中提取姿态角
void Mahony_Filter_Output(mahony_filter_t *mahony_filter)
{
  /*计算roll pitch yaw 欧拉角*/
  mahony_filter->pitch = -asinf(mahony_filter->rotation_matrix[2][0]);
  mahony_filter->roll = atan2f(mahony_filter->rotation_matrix[2][1],
			       mahony_filter->rotation_matrix[2][2]);
  mahony_filter->yaw = atan2f(mahony_filter->rotation_matrix[1][0],
			      mahony_filter->rotation_matrix[0][0]);
}

// 初始化Mahony滤波器的参数和函数指针
void Mahony_Filter_Init(mahony_filter_t *mahony_filter, float Kp, float Ki, float dt)
{
  mahony_filter->Kp = Kp;
  mahony_filter->Ki = Ki;
  mahony_filter->dt = dt;
  mahony_filter->q0 = 1;
  mahony_filter->Mahony_Filter_Input = Mahony_Filter_Input;
  mahony_filter->Mahony_Filter_Update = Mahony_Filter_Update;
  mahony_filter->Mahony_Filter_Output = Mahony_Filter_Output;
  mahony_filter->Mahony_Filter_RotationMatrix_Update = Mahony_Filter_RotationMatrix_Update;
}
/*-----------------------MahonyAHRS滤波器----------------------*/
/*-----------------------------------------------------------*/