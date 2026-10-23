#include <math.h>
#include "component/power/power.h"

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

float power_apply(power_channel_t *channels, uint8_t count, const power_model_t *model, float max_power_w)
{
    float predicted[16];
    float total = 0.0f;
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

        if (channels[index].output == NULL || predicted[index] <= 0.0f)
        {
            continue;
        }
        speed = channels[index].speed_rad_s;
        command = *channels[index].output;
        share = predicted[index] * (max_power_w / total);
        a = model->k_output2;
        b = model->k_output_speed * speed;
        c = model->k_speed2 * speed * speed + model->idle_w - share;
        if (a < 1.0e-12f)
        {
            *channels[index].output = command * (max_power_w / total);
            continue;
        }
        delta = b * b - 4.0f * a * c;
        if (delta < 0.0f)
        {
            *channels[index].output = command * (max_power_w / total);
            continue;
        }
        solved = (command >= 0.0f) ? ((-b + sqrtf(delta)) / (2.0f * a))
                                    : ((-b - sqrtf(delta)) / (2.0f * a));
        *channels[index].output = solved;
    }
    return total;
}
