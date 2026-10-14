#include "cdd/buzzer/buzzer.h"
#include "bsp/pwm/bsp_pwm.h"

void buzzer_on(uint32_t freq_hz)
{
    if (bsp_pwm_set_freq(PWM_BUZZER, freq_hz) == 0)
    {
        (void)bsp_pwm_set_duty(PWM_BUZZER, 500);
        (void)bsp_pwm_start(PWM_BUZZER);
    }
}

void buzzer_off(void)
{
    (void)bsp_pwm_stop(PWM_BUZZER);
}
