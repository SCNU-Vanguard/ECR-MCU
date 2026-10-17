#ifndef MEDIAN_H__
#define MEDIAN_H__
/* 定长中值滤波。窗口由调用者持有，初始化时用 0 填满，这些 0 也参与中值。 */
#include <stdint.h>

#define MEDIAN_MAX_LENGTH 100u

typedef struct
{
    float samples[MEDIAN_MAX_LENGTH];       /* 按数值从小到大排 */
    int age[MEDIAN_MAX_LENGTH];             /* 与 samples 一起移动，记下每个值进窗口时的序号 */
    uint8_t length;
    uint8_t next_age;                       /* 下一个要淘汰的最旧样本序号 */
    float median;
} median_filter_t;

/* length 超出上限时截断。零长度时 Add 直接返回，Get 返回 0 */
void median_init(median_filter_t *filter, uint8_t length);
void median_clear(median_filter_t *filter);
/* 更新 median。偶数窗口取排序后较大的那个中间值 */
void median_add(median_filter_t *filter, float sample);
/* lag=0 是最新样本。排序位置不是时间顺序，按进窗口的先后往回找。中值在 filter->median */
float median_get(median_filter_t *filter, uint16_t lag);
#endif
