#include "bsp/pwm/bsp_pwm.h"
#include "component/common/err.h"

static inline const bsp_pwm_cfg_t *cfg(pwm_id_t id)
{
    return ((unsigned)id < PWM_NUM) ? &board_pwm_cfg[id] : NULL;
}

int32_t bsp_pwm_start(pwm_id_t id)
{
    const bsp_pwm_cfg_t *c = cfg(id);
    if (c == NULL) { return ERR_INVAL; }
    return (HAL_TIM_PWM_Start(c->htim, c->channel) == HAL_OK) ? ERR_OK : ERR_IO;
}

int32_t bsp_pwm_stop(pwm_id_t id)
{
    const bsp_pwm_cfg_t *c = cfg(id);
    if (c == NULL) { return ERR_INVAL; }
    return (HAL_TIM_PWM_Stop(c->htim, c->channel) == HAL_OK) ? ERR_OK : ERR_IO;
}

int32_t bsp_pwm_set_duty(pwm_id_t id, uint32_t permille)
{
    const bsp_pwm_cfg_t *c = cfg(id);
    if (c == NULL) { return ERR_INVAL; }
    if (permille > 1000u) { permille = 1000u; }
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(c->htim);
    __HAL_TIM_SET_COMPARE(c->htim, c->channel, (uint32_t)(((uint64_t)(arr + 1u) * permille) / 1000u));
    return ERR_OK;
}

int32_t bsp_pwm_set_freq(pwm_id_t id, uint32_t hz)
{
    const bsp_pwm_cfg_t *c = cfg(id);
    if (c == NULL || hz == 0u) { return ERR_INVAL; }
    uint32_t psc = c->htim->Instance->PSC;
    uint32_t tick_hz = c->timer_clk_hz / (psc + 1u);
    if (hz > tick_hz) { return ERR_INVAL; }
    uint32_t old_arr = __HAL_TIM_GET_AUTORELOAD(c->htim);
    uint32_t old_ccr = __HAL_TIM_GET_COMPARE(c->htim, c->channel);
    uint32_t new_arr = tick_hz / hz - 1u;
    if (c->htim->Instance != TIM2 && new_arr > 0xFFFFu) { return ERR_INVAL; }   /* 16 位定时器放不下 */
    uint32_t new_ccr = (uint32_t)(((uint64_t)old_ccr * (new_arr + 1u)) / (old_arr + 1u));
    __HAL_TIM_SET_AUTORELOAD(c->htim, new_arr);
    __HAL_TIM_SET_COMPARE(c->htim, c->channel, new_ccr);
    return ERR_OK;
}
