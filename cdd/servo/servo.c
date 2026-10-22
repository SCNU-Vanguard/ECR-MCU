#include "cdd/servo/servo.h"
#include "bsp/pwm/bsp_pwm.h"
#include "bsp/include/bsp_ids.h"
#include "component/common/err.h"
#include "component/common/initcall.h"

static const pwm_id_e servo_pwm[SERVO_NUM] =
{
    [SERVO_0] = PWM_SERVO,
};

int32_t servo_set_pulse_us(servo_id_e id, uint16_t pulse_us)
{
    if ((unsigned)id >= SERVO_NUM)
    {
        return ERR_INVAL;
    }
    if (pulse_us < SERVO_PULSE_MIN)
    {
        pulse_us = SERVO_PULSE_MIN;
    }
    else if (pulse_us > SERVO_PULSE_MAX)
    {
        pulse_us = SERVO_PULSE_MAX;
    }
    /* 20 ms 周期、1 us 一格：百万分比 = 脉宽 / 周期 */
    return bsp_pwm_set_duty_ppm(servo_pwm[id], (uint32_t)pulse_us * (1000000u / SERVO_PERIOD_US));
}

int32_t servo_set_angle(servo_id_e id, float degree)
{
    float pulse;

    if (degree < 0.0f)
    {
        degree = 0.0f;
    }
    else if (degree > 180.0f)
    {
        degree = 180.0f;
    }
    pulse = (float)SERVO_PULSE_MIN + degree * ((float)(SERVO_PULSE_MAX - SERVO_PULSE_MIN) / 180.0f);
    return servo_set_pulse_us(id, (uint16_t)(pulse + 0.5f));
}

static int32_t servo_init(void)
{
    for (uint8_t index = 0; index < SERVO_NUM; index++)
    {
        (void)servo_set_pulse_us((servo_id_e)index, SERVO_PULSE_MID);
        (void)bsp_pwm_start(servo_pwm[index]);
    }
    return ERR_OK;
}
INIT_CDD_STAGE1(servo_init);
