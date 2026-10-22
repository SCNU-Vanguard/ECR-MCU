#ifndef CDD_SERVO_H__
#define CDD_SERVO_H__
#include "component/common/types.h"

/* 航模舵机。按编号调用，现在只接 SERVO_0（PE9，TIM1_CH1）。
 * 加一路：在 servo_id_e 里加编号，并在 servo.c 的表里填对应 PWM。
 * Cube 里 TIM1 预分频 240-1、周期 20000-1 之后，脉宽单位是微秒。
 * 上电停在中位 1500 us。 */

#define SERVO_PERIOD_US   20000u
#define SERVO_PULSE_MIN   500u
#define SERVO_PULSE_MAX   2500u
#define SERVO_PULSE_MID   1500u

typedef enum
{
    SERVO_0 = 0,
    SERVO_NUM
} servo_id_e;

/* 脉宽限制在 500..2500 us */
int32_t servo_set_pulse_us(servo_id_e id, uint16_t pulse_us);
/* 0..180 度线性对应 500..2500 us */
int32_t servo_set_angle(servo_id_e id, float degree);
#endif
