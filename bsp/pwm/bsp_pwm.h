#ifndef BSP_PWM_H__
#define BSP_PWM_H__
#include "bsp/include/bsp_hal.h"
#include "component/common/types.h"
#include "bsp/include/bsp_ids.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;            /* TIM_CHANNEL_x */
    uint32_t timer_clk_hz;       /* 定时器输入时钟：见 .ioc 的 TimxOutputFreq_Value（本工程 TIM3/TIM12 均为 240 MHz） */
} bsp_pwm_cfg_t;

extern const bsp_pwm_cfg_t board_pwm_cfg[PWM_NUM];       /* 由 board.c 提供 */

int32_t bsp_pwm_start(pwm_id_e id);
int32_t bsp_pwm_stop(pwm_id_e id);
/* 占空比 0..1000（千分比），按当前 ARR 换算 CCR */
int32_t bsp_pwm_set_duty(pwm_id_e id, uint32_t permille);
/* 占空比 0..1000000（百万分比），给需要细分辨率的场合（加热片只用 1% 左右的占空比） */
int32_t bsp_pwm_set_duty_ppm(pwm_id_e id, uint32_t ppm);
/* 改频率：保持 PSC 不变，只改 ARR，并按原占空比重算 CCR。适用于蜂鸣器变音调 */
int32_t bsp_pwm_set_freq(pwm_id_e id, uint32_t hz);
#endif
