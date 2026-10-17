#include "component/filter/moving_average/moving_average.h"

void moving_average_init(moving_average_filter_t *filter, uint8_t length)
{
    for (uint16_t i = 0; i < MOVING_AVERAGE_MAX_LENGTH; i++)
    {
        filter->samples[i] = 0.0f;
    }
    if (length > MOVING_AVERAGE_MAX_LENGTH)
    {
        length = MOVING_AVERAGE_MAX_LENGTH;
    }
    filter->length = length;
    filter->index = 0u;
    filter->average = 0.0f;
    filter->total = 0.0f;
}

void moving_average_add(moving_average_filter_t *filter, float sample)
{
    filter->total -= filter->samples[filter->index];
    filter->total += sample;
    filter->samples[filter->index] = sample;
    filter->average = filter->total / (float)filter->length;
    filter->index++;
    if (filter->index == filter->length)
    {
        filter->index = 0u;
    }
}

float moving_average_get(moving_average_filter_t *filter, uint16_t lag)
{
    uint8_t newest = (filter->index != 0u) ? (uint8_t)(filter->index - 1u) : (uint8_t)(filter->length - 1u);

    if (lag > filter->length)
    {
        lag = (uint16_t)(lag % filter->length);
    }
    if (lag > newest)
    {
        lag = (uint16_t)(filter->length + newest - lag);
    }
    else
    {
        lag = (uint16_t)(newest - lag);
    }
    return filter->samples[lag];
}

void moving_average_clear(moving_average_filter_t *filter)
{
    for (uint16_t i = 0; i < MOVING_AVERAGE_MAX_LENGTH; i++)
    {
        filter->samples[i] = 0.0f;
    }
    filter->index = 0u;
    filter->average = 0.0f;
    filter->total = 0.0f;
}

void moving_average_fill(moving_average_filter_t *filter, float value)
{
    for (uint16_t i = 0; i < filter->length; i++)
    {
        filter->samples[i] = value;
    }
    filter->index = 0u;
    filter->average = value;
    filter->total = value * (float)filter->length;
}
