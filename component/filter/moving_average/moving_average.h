#ifndef MOVING_AVERAGE_H__
#define MOVING_AVERAGE_H__
/* 定长滑动平均。窗口由调用者持有，初始化时用 0 填满，这些 0 也参与均值。 */
#include <stdint.h>

#define MOVING_AVERAGE_MAX_LENGTH 100u

typedef struct
{
    float samples[MOVING_AVERAGE_MAX_LENGTH];
    uint8_t length;
    uint8_t index;          /* 下一个要写入的位置 */
    float total;
    float average;
} moving_average_filter_t;

/* length 小于 1 不能用于 Add/Get；大于上限时截断到 MOVING_AVERAGE_MAX_LENGTH */
void moving_average_init(moving_average_filter_t *filter, uint8_t length);
void moving_average_clear(moving_average_filter_t *filter);
/* 用同一个值填满窗口，均值就是这个值 */
void moving_average_fill(moving_average_filter_t *filter, float value);
void moving_average_add(moving_average_filter_t *filter, float sample);
/* lag=0 是最新样本，按窗口循环往回取；不是返回 average */
float moving_average_get(moving_average_filter_t *filter, uint16_t lag);
#endif
