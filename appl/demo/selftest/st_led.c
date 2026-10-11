/* 自测：WS2812 状态灯。红 -> 绿 -> 蓝 循环，每种颜色 1 秒。 */
#include "osal/period.h"
#include "cdd/led/ws2812/ws2812.h"

static void led_task(void *arg)
{
    static uint32_t n;
    static const uint8_t color[3][3] = { {32, 0, 0}, {0, 32, 0}, {0, 0, 32} };   /* R G B，亮度 32/255 */
    (void)arg;
    const uint8_t *c = color[(n++ / 10u) % 3u];
    for (uint8_t i = 0; i < WS2812_NUM; i++) { ws2812_set(i, c[0], c[1], c[2]); }
    (void)ws2812_refresh();
}
PERIOD_TASK_DEFINE(st_led, 10, 12, 384, led_task, NULL);
