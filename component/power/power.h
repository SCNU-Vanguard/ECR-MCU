#ifndef COMPONENT_POWER_H__
#define COMPONENT_POWER_H__
#include "component/common/types.h"

/* 电机功率模型：P = k_output_speed * output * speed_rad_s
 *                 + k_speed2 * speed_rad_s^2
 *                 + k_output2 * output^2
 *                 + idle_w
 * output 用电机自己的最终控制量（大疆是电流原始值）。速度用 rad/s。 */

typedef struct
{
    float k_output_speed;
    float k_speed2;
    float k_output2;
    float idle_w;
} power_model_t;

/* 决定本周期允许多少瓦。任一来源无效就当没有。
 * 返回值 <= 0：不限制。 */
typedef struct
{
    bool     referee_valid;
    float    referee_limit_w;
    bool     supercap_valid;       /* 在线且没有不可恢复错误 */
    uint8_t  energy_raw;           /* 0..255 */
    float    supercap_limit_w;     /* 电容上报的底盘功率上限；没有裁判时用它 */
    float    extra_w;              /* 能量高于 energy_high_raw 时，在裁判上限上加这么多 */
    float    margin_w;             /* 能量不高于阈值时，从裁判上限里减这么多 */
    uint8_t  energy_high_raw;
} power_budget_t;

float power_budget_w(const power_budget_t *budget);

/* 一个参与限制的通道。output 指向该电机的最终控制量，函数会直接改它。 */
typedef struct
{
    float *output;
    float  speed_rad_s;
} power_channel_t;

/* 改各通道的 output，使预测总功率不超过 max_power_w。
 * max_power_w <= 0、count 为 0、model 为空：不改 output。
 * 负功率（再生）不削减。返回削减前的预测正功率。 */
float power_apply(power_channel_t *channels, uint8_t count, const power_model_t *model, float max_power_w);
#endif
