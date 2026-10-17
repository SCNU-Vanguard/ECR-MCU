#include "component/filter/median/median.h"

void median_add(median_filter_t *filter, float sample)
{
    uint16_t slot;

    if (filter->length == 0u)
    {
        return;
    }
    for (slot = 0u; slot < filter->length; slot++)
    {
        if (filter->age[slot] == filter->next_age)
        {
            break;
        }
    }
    /* next_age 是要淘汰的最旧样本。挪数值时年龄索引一起挪。
     * 先看边界再读相邻项，长度为 1 或插在两端时不会读到窗口外。 */
    if (slot == filter->length)
    {
        return;
    }
    while (slot + 1u < filter->length && filter->samples[slot + 1u] < sample)
    {
        filter->samples[slot] = filter->samples[slot + 1u];
        filter->age[slot] = filter->age[slot + 1u];
        slot++;
    }
    while (slot > 0u && filter->samples[slot - 1u] > sample)
    {
        filter->samples[slot] = filter->samples[slot - 1u];
        filter->age[slot] = filter->age[slot - 1u];
        slot--;
    }
    filter->samples[slot] = sample;
    filter->age[slot] = filter->next_age;
    filter->next_age++;
    if (filter->next_age == filter->length)
    {
        filter->next_age = 0u;
    }
    filter->median = filter->samples[filter->length / 2u];
}

float median_get(median_filter_t *filter, uint16_t lag)
{
    uint16_t age;
    uint16_t index;

    if (filter->length == 0u)
    {
        return 0.0f;
    }
    age = (uint16_t)(lag % filter->length);
    index = (uint16_t)((filter->next_age + filter->length - 1u - age) % filter->length);
    for (uint16_t i = 0u; i < filter->length; i++)
    {
        if (filter->age[i] == (int)index)
        {
            return filter->samples[i];
        }
    }
    return 0.0f;
}

void median_init(median_filter_t *filter, uint8_t length)
{
    for (uint16_t i = 0u; i < MEDIAN_MAX_LENGTH; i++)
    {
        filter->samples[i] = 0.0f;
        filter->age[i] = (int)i;
    }
    if (length > MEDIAN_MAX_LENGTH)
    {
        length = MEDIAN_MAX_LENGTH;
    }
    filter->length = length;
    filter->next_age = 0u;
    filter->median = filter->samples[filter->length / 2u];
}

void median_clear(median_filter_t *filter)
{
    for (uint16_t i = 0u; i < MEDIAN_MAX_LENGTH; i++)
    {
        filter->samples[i] = 0.0f;
        filter->age[i] = (int)i;
    }
    filter->median = filter->samples[filter->length / 2u];
    filter->next_age = 0u;
}
