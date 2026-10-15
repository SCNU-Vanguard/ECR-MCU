/**
 ******************************************************************************
 * @file	 user_lib.h
 * @author  Wang Hongxi
 * @author  GUATAI
 * @version V1.0.0
 * @date    2021/2/18
 * @brief 常用数值约束、斜坡、滤波与坐标变换工具。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef __USER_LIB_H__
#define __USER_LIB_H__

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "arm_math.h"
#include "FreeRTOS.h"                /* pvPortMalloc */

/* 动态内存统一走 FreeRTOS 堆（heap_4，configTOTAL_HEAP_SIZE），分配失败会触发 vApplicationMallocFailedHook。没有释放接口：只在初始化时分配 */
#ifndef user_malloc
#define user_malloc pvPortMalloc
#endif

#define FAST_CALC 0

#ifndef user_abs
#define user_abs( x ) ((x > 0) ? x : -x)
#endif

#define msin(x) (arm_sin_f32(x))
#define mcos(x) (arm_cos_f32(x))

typedef arm_matrix_instance_f32 mat;
// 若运算速度不够,可以使用q31代替f32,但是精度会降低
#if FAST_CALC
#define MatAdd arm_mat_add_q31
#define MatSubtract arm_mat_sub_q31
#define MatMultiply arm_mat_mult_q31
#define MatTranspose arm_mat_trans_q31
#define MatInverse arm_mat_inverse_q31
#else
#define MatAdd arm_mat_add_f32
#define MatSubtract arm_mat_sub_f32
#define MatMultiply arm_mat_mult_f32
#define MatTranspose arm_mat_trans_f32
#define MatInverse arm_mat_inverse_f32
#endif
void MatInit(mat *m, uint8_t row, uint8_t col);

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

typedef enum
{
	SLOPE_FIRST_REAL   = 0,
	SLOPE_FIRST_TARGET = 1,
} ramp_strategy_e;

typedef struct
{
	float min_value;    //限幅最小值
	float max_value;    //限幅最大值
	float frame_period; //时间间隔

	float increase_value;
	float decrease_value;

	ramp_strategy_e ramp_state;
} __attribute__((__packed__)) ramp_init_config_t;

typedef struct
{
	float target;       //输入数据
	float out;          //输出数据
	float min_value;    //限幅最小值
	float max_value;    //限幅最大值
	float frame_period; //时间间隔

    float last_target;
    float change_reset_flag;

	float plan_value;
	float real_value;
	float increase_value;
	float decrease_value;
	uint32_t DWT_CNT;
	float dt;

	ramp_strategy_e ramp_state;
} __attribute__((__packed__)) ramp_function_source_t;

/**
 * @brief 返回一块干净的内存,不过仍然需要强制转换为你需要的类型
 *
 * @param size 分配大小
 * @return void*
 */
void *zmalloc(size_t size);

//快速开方
float Sqrt(float x);

//斜波函数初始化
ramp_function_source_t *ramp_init(ramp_init_config_t *config);

//斜波函数计算
float ramp_calc(ramp_function_source_t *ramp_source_type, float input);

//绝对限制
float abs_limit(float num, float Limit);

//判断符号位
float sign(float value);

//浮点死区
float float_deadband(float Value, float minValue, float maxValue);

//限幅函数
float float_constrain(float Value, float minValue, float maxValue);

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);

//循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue);

float trans_zeropoint(float input, float middle_zero_point, float measure_min_value, float measure_max_value, float target_min_value, float target_max_value);

float trans_thresholds(float input, float measure_min_value, float measure_max_value, float target_min_value, float target_max_value);

//角度 °限幅 180 ~ -180
float theta_format(float Ang);

int float_rounding(float raw);

float *Norm3d(float *v);

float NormOf3d(float *v);

void Cross3d(float *v1, float *v2, float *res);

float Dot3d(float *v1, float *v2);

float AverageFilter(float new_data, float *buf, uint8_t len);

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

//判断是否出现NAN
bool Judge_IF_NAN(float x );

typedef struct
{
	float y;
	float alpha_min;
	float alpha_max;
	float k;
	float e0;
	float rate_up;
	float rate_down;
	uint32_t dwt_cnt;
/* 该结构保存运行时状态，不参与通信封包。保持自然对齐，因为 dwt_cnt 会以地址
 * 形式传给 DWT 计时函数；强制紧凑排列可能造成非对齐访问。 */
} logistic_smooth_t;

void LogisticSmooth_Init(logistic_smooth_t *s, float y0, float alpha_min, float alpha_max, float k, float e0, float rate_up, float rate_down);
float LogisticSmooth_UpdateDWT(logistic_smooth_t *s, float target);

typedef struct
{
	float y;           // 当前输出值
	float y_start;     // 当前段起始值
	float y_target;    // 当前段目标值
	float t;           // 当前时间（从0开始）
	float T_up;        // 上升段总时间
	float T_down;      // 下降段总时间
	float target_eps;  // 目标变化阈值
	float p;
	float delta_y;
	float u;
	float T;
	float last_target; // 上次的目标值
	uint32_t dwt_cnt;  // DWT计时器计数值
}scurve_smooth_t;

void SCurveSmooth_Init(scurve_smooth_t *s, float y0, float T_up, float T_down, float target_eps);
float SCurveSmooth_UpdateDWT(scurve_smooth_t *s, float target);

#endif /* __USER_LIB_H__ */
