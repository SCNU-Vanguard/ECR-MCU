#ifndef POWER_CONFIG_H__
#define POWER_CONFIG_H__
/* 参与功率控制的电机。只列在这里的电机会被改 output，没列出的原样发送。
 * 系数来自 2026 步兵 M3508 模型。原式用转子 rpm，这里改成 rad/s：
 *   k_output_speed = 1.996889994e-6 * 9.5493
 *   k_speed2       = 2.0326e-7 * 9.5493^2
 * 能量高于 38（约 15%）时允许比裁判上限多 25 W，否则少 10 W。 */
#define POWER_MOTOR_COUNT  4u

#define POWER_MODEL_INIT \
    { 1.9069e-5f, 1.8535e-5f, 1.453e-7f, 0.75f }

#define POWER_EXTRA_W          25.0f
#define POWER_MARGIN_W         10.0f
#define POWER_ENERGY_HIGH_RAW  38u
#endif
