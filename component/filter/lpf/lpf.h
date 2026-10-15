/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : lpf.h
 * @brief          : 一阶、二阶递推低通滤波
 * @author         : Yan Yuanbin
 * @date           : 2023/04/27
 * @version        : v1.0
 ******************************************************************************
 * @attention      : To be perfected
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
/* Exported types ------------------------------------------------------------*/
/**
 * @brief  一阶低通滤波器信息的结构体
 */
typedef struct
{
	bool Initialized; // 初始化标志 0:未初始化 1：初始化
	float Input;      // 当前输入
	float Output;     // 输出
	float Alpha;      // 滤波系数
}lowpass_filter1p_info_t;

/**
 * @brief 二阶低通滤波器信息的结构体.
 */
typedef struct
{
	bool Initialized; // 初始化标志 0:未初始化 1：初始化
	float Input;      // 当前输入
	float Output[3];  // 输出 现在和过去两次的输出
	float Alpha[3];   // 二阶滤波器系数
}lowpass_filter2p_info_t;

/* Extern Functions Prototypes ---------------------------------------------*/

/**
 * @brief 初始化非空实例，Alpha 为上一输出权重，推荐 0～1，越大滞后越强。
 */
extern void LowPass_Filter1p_Init( lowpass_filter1p_info_t *LPF, float Alpha );

/**
 * @brief 输入与输出同单位；仅接受 (-65535, 65535) 内输入，否则保持旧输出。
 * @note 先 Init，每个样本调用一次，实例由调用者持有且不可并发更新。
 */
extern float LowPass_Filter1p_Update( lowpass_filter1p_info_t *lpf, float input );

/**
 * @brief 复制三个系数；lpf 和 alpha 非空，实例需先整体清零。
 * @note Init 当前不重置 Initialized；重复初始化前也需自行清零该标志。
 */
extern void LowPass_Filter2p_Init( lowpass_filter2p_info_t *lpf, float alpha[3] );

/**
 * @brief y=alpha[0]*y1+alpha[1]*y2+alpha[2]*input，系数稳定性由调用者保证。
 * @note 仅接受 (-1000, 1000) 内输入，否则保持旧输出；不可并发更新。
 */
extern float LowPass_Filter2p_Update( lowpass_filter2p_info_t *lpf, float input );

#ifdef __cplusplus
}
#endif

#endif // LOWPASS_FILTER_H
