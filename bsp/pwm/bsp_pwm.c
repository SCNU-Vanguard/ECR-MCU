#include "bsp/pwm/bsp_pwm.h"
#include "component/common/err.h"

static inline const bsp_pwm_cfg_t *pwm_config(pwm_id_e id)
{
    return ((unsigned)id < PWM_NUM) ? &board_pwm_cfg[id] : NULL;
}

int32_t bsp_pwm_start(pwm_id_e id)
{
    const bsp_pwm_cfg_t *config = pwm_config(id);

    if (config == NULL)
    {
        return ERR_INVAL;
    }
    return (HAL_TIM_PWM_Start(config->htim, config->channel) == HAL_OK) ? ERR_OK : ERR_IO;
}

int32_t bsp_pwm_stop(pwm_id_e id)
{
    const bsp_pwm_cfg_t *config = pwm_config(id);

    if (config == NULL)
    {
        return ERR_INVAL;
    }
    return (HAL_TIM_PWM_Stop(config->htim, config->channel) == HAL_OK) ? ERR_OK : ERR_IO;
}

int32_t bsp_pwm_set_duty(pwm_id_e id, uint32_t permille)
{
    const bsp_pwm_cfg_t *config = pwm_config(id);

    if (config == NULL)
    {
        return ERR_INVAL;
    }
    if (permille > 1000u)
    {
        permille = 1000u;
    }
    uint32_t auto_reload = __HAL_TIM_GET_AUTORELOAD(config->htim);

    __HAL_TIM_SET_COMPARE(config->htim, config->channel, (uint32_t)(((uint64_t)(auto_reload + 1u) * permille) / 1000u));
    return ERR_OK;
}

int32_t bsp_pwm_set_duty_ppm(pwm_id_e id, uint32_t ppm)
{
    const bsp_pwm_cfg_t *config = pwm_config(id);

    if (config == NULL)
    {
        return ERR_INVAL;
    }
    if (ppm > 1000000u)
    {
        ppm = 1000000u;
    }
    uint32_t auto_reload = __HAL_TIM_GET_AUTORELOAD(config->htim);

    __HAL_TIM_SET_COMPARE(config->htim, config->channel, (uint32_t)(((uint64_t)(auto_reload + 1u) * ppm) / 1000000u));
    return ERR_OK;
}

int32_t bsp_pwm_set_freq(pwm_id_e id, uint32_t hz)
{
    const bsp_pwm_cfg_t *config = pwm_config(id);

    if (config == NULL || hz == 0u)
    {
        return ERR_INVAL;
    }
    uint32_t prescaler = config->htim->Instance->PSC;
    uint32_t tick_hz = config->timer_clk_hz / (prescaler + 1u);

    if (hz > tick_hz)
    {
        return ERR_INVAL;
    }
    uint32_t old_auto_reload = __HAL_TIM_GET_AUTORELOAD(config->htim);
    uint32_t old_compare = __HAL_TIM_GET_COMPARE(config->htim, config->channel);
    uint32_t new_auto_reload = tick_hz / hz - 1u;

    if (config->htim->Instance != TIM2 && new_auto_reload > 0xFFFFu)    /* 16 位定时器放不下 */
    {
        return ERR_INVAL;
    }
    uint32_t new_compare = (uint32_t)(((uint64_t)old_compare * (new_auto_reload + 1u)) / (old_auto_reload + 1u));

    __HAL_TIM_SET_AUTORELOAD(config->htim, new_auto_reload);
    __HAL_TIM_SET_COMPARE(config->htim, config->channel, new_compare);
    return ERR_OK;
}
