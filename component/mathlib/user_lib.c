/**
 ******************************************************************************
 * @file	 user_lib.c
 * @author  Wang Hongxi
 * @author  modified by neozng
 * @author  GUATAI
 * @version 0.2 beta
 * @date    2021/2/18
 * @brief 常用数值约束、斜坡、滤波与坐标变换工具。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "component/mathlib/user_lib.h"
#include "bsp/tick/bsp_tick.h"

/**
 * @brief          斜波函数初始化
 * @author         GUATAI
 * @param[in]      斜波函数结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      最大值
 * @param[in]      最小值
 * @retval         返回空
 */
ramp_function_source_t *ramp_init(ramp_init_config_t *config)
{
	ramp_function_source_t *ramp = (ramp_function_source_t *) user_malloc(sizeof(ramp_function_source_t));
	memset(ramp, 0, sizeof(ramp_function_source_t));

	ramp->frame_period   = config->frame_period;
	ramp->max_value      = config->max_value;
	ramp->min_value      = config->min_value;
	ramp->increase_value = config->increase_value;
	ramp->decrease_value = config->decrease_value;
	ramp->ramp_state     = config->ramp_state;
	ramp->target         = 0.0f;
	ramp->out            = 0.0f;

	return ramp;
}

/**
 * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
 * @author         GUATAI
 * @param[in]      斜波函数结构体
 * @param[in]      输入值
 * @retval         返回空
 */
//TODO:(GUATAI)可以考虑利用定时器来进行控制器目标值更新
float ramp_calc(ramp_function_source_t *ramp, float input)
{
	ramp->target = input;
	if (ramp->ramp_state == SLOPE_FIRST_REAL)
	{
		if(ramp->change_reset_flag == 1)
		{
			ramp->out = ramp->real_value;
			ramp->plan_value = ramp->real_value;
			ramp->change_reset_flag = 0;
		}

		if ((ramp->target >= ramp->real_value && ramp->real_value >= ramp->plan_value) || (ramp->target <= ramp->real_value && ramp->real_value <= ramp->plan_value))
		{
			ramp->out = ramp->real_value;
			ramp->plan_value = ramp->real_value;
		}
		if (ramp->plan_value > 0.0f)
		{
			//正值加速
			if (ramp->target > ramp->plan_value)
			{
				if (user_abs((ramp->plan_value - ramp->target)) > ramp->increase_value)
				{
					ramp->out += ramp->increase_value;
				}
				else
				{
					ramp->out = ramp->target;
				}
			}
			//正值减速
			else if (ramp->target < ramp->plan_value)
			{
				if (user_abs((ramp->plan_value - ramp->target)) > ramp->decrease_value)
				{
					ramp->out -= ramp->decrease_value;
				}
				else
				{
					ramp->out = ramp->target;
				}
			}
		}
		else if (ramp->plan_value < 0.0f)
		{
			//负值加速
			if (ramp->target < ramp->plan_value)
			{
				if (user_abs((ramp->plan_value - ramp->target)) > ramp->increase_value)
				{
					ramp->out -= ramp->increase_value;
				}
				else
				{
					ramp->out = ramp->target;
				}
			}
			//负值减速
			else if (ramp->target > ramp->plan_value)
			{
				if (user_abs((ramp->plan_value - ramp->target)) > ramp->decrease_value)
				{
					ramp->out += ramp->decrease_value;
				}
				else
				{
					ramp->out = ramp->target;
				}
			}
		}
		else
		{
			if (ramp->target > ramp->plan_value)
			{
				//0值正加速
				if (user_abs((ramp->plan_value - ramp->target)) > ramp->increase_value)
				{
					ramp->out += ramp->increase_value;
				}
				else
				{
					ramp->out = ramp->target;
				}
			}
			else if (ramp->target < ramp->plan_value)
			{
				//0值负加速
				if (user_abs((ramp->plan_value - ramp->target)) > ramp->increase_value)
				{
					ramp->out -= ramp->increase_value;
				}
				else
				{
					ramp->out = ramp->target;
				}
			}
		}
		ramp->plan_value = ramp->out;
	}
	else if (ramp->ramp_state == SLOPE_FIRST_TARGET)
	{
		ramp->out += ramp->target * ramp->frame_period;
		if (ramp->out > ramp->max_value)
		{
			ramp->out = ramp->max_value;
		}
		else if (ramp->out < ramp->min_value)
		{
			ramp->out = ramp->min_value;
		}
	}
	return ramp->out;
}

void *zmalloc(size_t size)
{
	void *ptr = pvPortMalloc(size);
	memset(ptr, 0, size);
	return ptr;
}

// 快速开方
float Sqrt(float x)
{
	float y;
	float delta;
	float maxError;

	if (x <= 0)
	{
		return 0;
	}

	// initial guess
	y = x / 2;

	// refine
	maxError = x * 0.001f;

	do
	{
		delta = (y * y) - x;
		y -= delta / (2 * y);
	} while (delta > maxError || delta < -maxError);

	return y;
}

// 绝对值限制
float abs_limit(float num, float Limit)
{
	if (num > Limit)
	{
		num = Limit;
	}
	else if (num < -Limit)
	{
		num = -Limit;
	}
	return num;
}

// 判断符号位
float sign(float value)
{
	if (value >= 0.0f)
	{
		return 1.0f;
	}
	else
	{
		return -1.0f;
	}
}

// 浮点死区
float float_deadband(float Value, float minValue, float maxValue)
{
	if (Value < maxValue && Value > minValue)
	{
		Value = 0.0f;
	}
	return Value;
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
	if (Value < minValue)
	{
		return minValue;
	}
	else if (Value > maxValue)
	{
		return maxValue;
	}
	else
	{
		return Value;
	}
}

// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
	if (Value < minValue)
	{
		return minValue;
	}
	else if (Value > maxValue)
	{
		return maxValue;
	}
	else
	{
		return Value;
	}	
}

// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue)
{
	if (maxValue < minValue)
	{
		return Input;
	}

	if (Input > maxValue)
	{
		float len = maxValue - minValue;
		while (Input > maxValue)
		{
			Input -= len;
		}
	}
	else if (Input < minValue)
	{
		float len = maxValue - minValue;
		while (Input < minValue)
		{
			Input += len;
		}
	}
	return Input;
}

float trans_zeropoint(float input, float middle_zero_point, float measure_min_value, float measure_max_value, float target_min_value, float target_max_value)
{
	float k_input = 0.0f;
	float measure_temp = 0.0f;
	float target_temp = 0.0f;

	input = input - (middle_zero_point - (measure_max_value - measure_min_value) / 2.0f);

	input = loop_float_constrain(input, measure_min_value, measure_max_value);

	measure_temp = measure_max_value - measure_min_value;
	if(measure_temp != 0.0f)
	{
		k_input = input / measure_temp;
	}
	if(k_input < 0.0f )
	{
		k_input = 0.0f;
	}
	target_temp = k_input * (target_max_value - target_min_value);

	k_input = target_min_value + target_temp;

	return k_input;
}

float trans_thresholds(float input, float measure_min_value, float measure_max_value, float target_min_value, float target_max_value)
{
	float k_input = 0.0f;
	float measure_temp = 0.0f;
	float target_temp = 0.0f;

	input = input - measure_min_value;

	measure_temp = measure_max_value - measure_min_value;
	if(measure_temp != 0.0f)
	{
		k_input = input / measure_temp;
	}
	if(k_input < 0.0f )
	{
		k_input = 0.0f;
	}
	target_temp = k_input * (target_max_value - target_min_value);

	k_input = target_min_value + target_temp;

	return k_input;
}

// 弧度格式化为-PI~PI

// 角度格式化为-180~180
float theta_format(float Ang)
{
	return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
	static int integer;
	static float decimal;
	integer = (int) raw;
	decimal = raw - integer;
	if (decimal > 0.5f)
		integer++;
	return integer;
}

// 三维向量归一化
float *Norm3d(float *v)
{
	float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	v[0] /= len;
	v[1] /= len;
	v[2] /= len;
	return v;
}

// 计算模长
float NormOf3d(float *v)
{
	return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 三维向量叉乘v1 x v2
void Cross3d(float *v1, float *v2, float *res)
{
	res[0] = v1[1] * v2[2] - v1[2] * v2[1];
	res[1] = v1[2] * v2[0] - v1[0] * v2[2];
	res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float *v1, float *v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

void MatInit(mat *m, uint8_t row, uint8_t col)
{
	m->numCols = col;
	m->numRows = row;
	m->pData   = (float *) zmalloc(row * col * sizeof(float));
}

//判断是否出现NAN
bool Judge_IF_NAN(float x)
{
	bool res = 0;
	res      = (bool) isnan(x);
	return res;
}

void LogisticSmooth_Init(logistic_smooth_t *s, float y0, float alpha_min, float alpha_max, float k, float e0, float rate_up, float rate_down)
{
	if (s == NULL)
	{
		return;
	}

	s->y = y0;
	s->alpha_min = alpha_min;
	s->alpha_max = alpha_max;
	s->k = k;
	s->e0 = e0;
	s->rate_up = rate_up;
	s->rate_down = rate_down;
	s->dwt_cnt = bsp_cycle_count();
}

static float LogisticSmooth_Update(logistic_smooth_t *s, float target, float dt)
{
	if (s == NULL)
	{
		return target;
	}

	if (!(dt > 0.0f && dt < 1.0f))
	{
		dt = 0.0f;
	}

	float e = target - s->y;
	float eabs = user_abs(e);

	float w;
	if (s->k == 0.0f)
	{
		w = 1.0f;
	}
	else
	{
		float z = -s->k * (eabs - s->e0);
		w = 1.0f / (1.0f + expf(z));
	}

	float alpha = s->alpha_min + (s->alpha_max - s->alpha_min) * w;
	alpha = float_constrain(alpha, 0.0f, 1.0f);

	float y_next = s->y + alpha * e;

	if (dt > 0.0f)
	{
		float dy = y_next - s->y;
		if (dy > 0.0f && s->rate_up > 0.0f)
		{
			float dy_max = s->rate_up * dt;
			if (dy > dy_max)
			{
				y_next = s->y + dy_max;
			}
		}
		else if (dy < 0.0f && s->rate_down > 0.0f)
		{
			float dy_max = s->rate_down * dt;
			if (-dy > dy_max)
			{
				y_next = s->y - dy_max;
			}
		}
	}

	s->y = y_next;
	return s->y;
}

float LogisticSmooth_UpdateDWT(logistic_smooth_t *s, float target)
{
	if (s == NULL)
	{
		return target;
	}

	float dt = bsp_cycle_delta_s(&s->dwt_cnt);
	return LogisticSmooth_Update(s, target, dt);
}

void SCurveSmooth_Init(scurve_smooth_t *s, float y0, float T_up, float T_down, float target_eps)
{
	if (s == NULL)
	{
		return;
	}

	s->y = y0;
	s->y_start = y0;
	s->y_target = y0;
	s->last_target = y0;
	s->t = 0.0f;
	s->T_up = T_up;
	s->T_down = T_down;
	s->target_eps = target_eps;
	// 检测目标是否真的变化了
	s->target_eps = (s->target_eps > 0.0f) ? s->target_eps : 0.0f;
	s->dwt_cnt = bsp_cycle_count();
}

static float SCurveSmooth_Update(scurve_smooth_t *s, float target, float dt)
{

	if (!(dt > 0.0f && dt < 1.0f))
	{
		dt = 0.0f;
	}
	
	s->y_target = target;
	
	float target_diff = fabs(s->y_target - s->last_target);
	
	if (target_diff > 0.0f)
	{
		// 目标变化，从当前值重新规划
		s->y_start = s->y;
		s->y_target = target;
		s->last_target = s->y_target;
		s->t = 0.0f;
		// 计算当前段的总变化量（带符号）
		s->delta_y = s->y_target - s->y_start;
			// 根据变化方向选择时间参数
		s->T = (s->delta_y >= 0.0f) ? s->T_up : s->T_down;
	}
	
	// 如果变化量很小，直接到达目标
	if (user_abs(s->delta_y) <= s->target_eps)
	{
		s->y = s->y_target;
		return s->y;
	}

	if (s->T < 0.0f)
	{
		s->y = s->y_target;
		return s->y;
	}

	// 更新时间
	if (dt > 0.0f)
	{
		s->t += dt;
		if (s->t > s->T)
		{
			s->t = s->T;
		}
	}
	else
	{
		return s->y;
	}

	// 计算S曲线输出
	s->u = s->t / s->T;
	if (s->u <= 0.0f)
	{
		s->p = 0.0f;
	}
	else if (s->u >= 1.0f)
	{
		s->p = 1.0f;
	}
	else
	{
		s->p = s->u * s->u * (3.0f - 2.0f * s->u);
	}
	s->y = s->y_start + s->delta_y * s->p;
	return s->y;
}

float SCurveSmooth_UpdateDWT(scurve_smooth_t *s, float target)
{
	if (s == NULL)
	{
		return target;
	}

	float dt = bsp_cycle_delta_s(&s->dwt_cnt);
	return SCurveSmooth_Update(s, target, 0.001f);
}

// float Euler_Number = 0.0f;

// /**
//  * @brief Calculate the floating-point logistic curves.
//  * @param x: the curves input variables
//  * @param k: the curves slope
//  * @param x0: the curves phase
//  * @note y = 1/(1+e^(-k*(x-x0)))
//  *       k > 0: 1->0
//  *       k < 0: 0->1
//  * @retval the curves output
//  */
// float LogisticCurves_Calc(float x , float k ,float x0)
// {
// 	float y = 0.f;
	
// 	if(k == 0.f)return 1.f;
	
// 	y = 1/(1+pow(Euler_Number,(k*(x-x0))));
	
// 	return y;
// }