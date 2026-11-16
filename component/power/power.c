#include <math.h>
#include "component/power/power.h"

#define POWER_RECHECK_MAX   4u      /* 复核后整体缩放的最大次数 */

static float predict(const power_model_t *model, float output, float speed_rad_s)
{
    return model->k_output_speed * output * speed_rad_s
         + model->k_speed2 * speed_rad_s * speed_rad_s
         + model->k_output2 * output * output
         + model->idle_w;
}

float power_budget_w(const power_budget_t *budget)
{
    float limit;

    if (budget == NULL || (!budget->referee_valid && !budget->supercap_valid))
    {
        return 0.0f;
    }
    if (!budget->referee_valid)
    {
        return budget->supercap_limit_w;
    }
    limit = budget->referee_limit_w;
    if (budget->supercap_valid)
    {
        if (budget->energy_raw > budget->energy_high_raw)
        {
            limit += budget->extra_w;
        }
        else
        {
            limit -= budget->margin_w;
        }
    }
    return (limit > 0.0f) ? limit : 0.0f;
}

/* 合法根：与指令同号，且幅值不超过指令（只允许削减，不允许反向或放大） */
static bool root_valid(float root, float command)
{
    return (root * command > 0.0f) && (fabsf(root) <= fabsf(command));
}

/* 所有通道预测功率里为正的部分之和 */
static float positive_total(const power_channel_t *channels, uint8_t count, const power_model_t *model)
{
    float total = 0.0f;

    for (uint8_t index = 0; index < count; index++)
    {
        float power;

        if (channels[index].output == NULL)
        {
            continue;
        }
        power = predict(model, *channels[index].output, channels[index].speed_rad_s);
        if (power > 0.0f)
        {
            total += power;
        }
    }
    return total;
}

/* 把预测功率为正的通道的 output 按 scale 缩小（scale 在 0..1） */
static void scale_positive(power_channel_t *channels, uint8_t count, const power_model_t *model, float scale)
{
    for (uint8_t index = 0; index < count; index++)
    {
        if (channels[index].output == NULL)
        {
            continue;
        }
        if (predict(model, *channels[index].output, channels[index].speed_rad_s) > 0.0f)
        {
            *channels[index].output *= scale;
        }
    }
}

float power_apply(power_channel_t *channels, uint8_t count, const power_model_t *model, float max_power_w)
{
    float predicted[16];
    float total = 0.0f;
    float scale;
    uint8_t index;

    if (channels == NULL || model == NULL || count == 0u || max_power_w <= 0.0f)
    {
        return 0.0f;
    }
    if (count > 16u)
    {
        count = 16u;
    }
    for (index = 0; index < count; index++)
    {
        predicted[index] = 0.0f;
        if (channels[index].output == NULL)
        {
            continue;
        }
        predicted[index] = predict(model, *channels[index].output, channels[index].speed_rad_s);
        if (predicted[index] > 0.0f)
        {
            total += predicted[index];
        }
    }
    if (total <= max_power_w || total <= 0.0f)
    {
        return total;
    }
    scale = max_power_w / total;                                /* 0..1：无合法根时的退路，只向 0 缩 */
    for (index = 0; index < count; index++)
    {
        float speed;
        float command;
        float share;
        float a;
        float b;
        float c;
        float delta;
        float solved;
        float root;

        if (channels[index].output == NULL || predicted[index] <= 0.0f)
        {
            continue;
        }
        speed = channels[index].speed_rad_s;
        command = *channels[index].output;
        share = predicted[index] * scale;
        a = model->k_output2;
        b = model->k_output_speed * speed;
        c = model->k_speed2 * speed * speed + model->idle_w - share;
        solved = command * scale;
        if (a < 1.0e-12f)
        {
            if (fabsf(b) > 1.0e-12f)                            /* 二次项可忽略：解一次方程 b*x + c = 0 */
            {
                root = -c / b;
                if (root_valid(root, command))
                {
                    solved = root;
                }
            }
        }
        else
        {
            delta = b * b - 4.0f * a * c;
            if (delta >= 0.0f)
            {
                float root_hi = (-b + sqrtf(delta)) / (2.0f * a);
                float root_lo = (-b - sqrtf(delta)) / (2.0f * a);
                bool found = false;

                /* 两个根之间功率不超过 share；取合法根里幅值最大（离指令最近）的 */
                if (root_valid(root_hi, command))
                {
                    solved = root_hi;
                    found = true;
                }
                if (root_valid(root_lo, command) && (!found || fabsf(root_lo) > fabsf(solved)))
                {
                    solved = root_lo;
                }
            }
        }
        *channels[index].output = solved;
    }
    /* 复核：空转项、速度项或根选取不理想时可能仍超预算，整体按比例再缩，次数有上限 */
    for (index = 0; index < POWER_RECHECK_MAX; index++)
    {
        float after = positive_total(channels, count, model);

        if (after <= max_power_w || after <= 0.0f)
        {
            break;
        }
        scale_positive(channels, count, model, max_power_w / after);
    }
    return total;
}
