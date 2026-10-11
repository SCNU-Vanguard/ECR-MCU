/* demo：板载 WS2812 灯，1 kHz 周期任务，每个周期改一次颜色（色相缓慢循环）。
 * 每次都 ws2812_refresh()：SPI6 轮询发送一帧（1 颗灯约 85 us），所以灯多了要改成 DMA。 */
#include "osal/period.h"
#include "cdd/led/ws2812/ws2812.h"

#include CMSIS_device_header       /* DWT、SystemCoreClock */

#define LED_BRIGHT      32u        /* 亮度上限 0..255，板载灯很亮，不要开满 */
#define HUE_STEPS       3000u      /* 一圈色相对应的周期数：1 kHz 下约 3 秒转一圈 */

/* 色相 -> RGB：把一圈分成 6 段，每段内一个通道线性变化，亮度由 LED_BRIGHT 限制 */
static void hue_to_rgb(uint32_t h, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint32_t seg  = h * 6u / HUE_STEPS;                          /* 0..5 */
    uint32_t frac = (h * 6u % HUE_STEPS) * LED_BRIGHT / HUE_STEPS;  /* 段内 0..LED_BRIGHT */
    uint8_t up = (uint8_t)frac, down = (uint8_t)(LED_BRIGHT - frac), full = LED_BRIGHT;
    switch (seg) {
    case 0:  *r = full; *g = up;   *b = 0;    break;
    case 1:  *r = down; *g = full; *b = 0;    break;
    case 2:  *r = 0;    *g = full; *b = up;   break;
    case 3:  *r = 0;    *g = down; *b = full; break;
    case 4:  *r = up;   *g = 0;    *b = full; break;
    default: *r = full; *g = 0;    *b = down; break;
    }
}

static void led_ctrl_run(void *arg)
{
    static uint32_t hue;
    uint8_t r, g, b;
    (void)arg;
    hue_to_rgb(hue, &r, &g, &b);
    if (++hue >= HUE_STEPS) { hue = 0; }
    ws2812_set(0, r, g, b);
    (void)ws2812_refresh();
}
/* 任务名 led_ctrl_task，1000 Hz，优先级 3（数值小优先级高），栈 256 字 */
PERIOD_TASK_DEFINE(led_ctrl_task, 1000, 3, 256, led_ctrl_run, NULL);
