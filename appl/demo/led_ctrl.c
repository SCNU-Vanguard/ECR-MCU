/* demo：板载 WS2812 灯，以 1 Hz 亮灭（亮 500 ms、灭 500 ms）。
 * 直接用任务周期实现：任务频率 = 亮灭频率 x 2，每次运行翻转一次。 */
#include "osal/period.h"
#include "cdd/led/ws2812/ws2812.h"

#define LED_BRIGHT          32u        /* 亮度 0..255，板载灯很亮，不要开满 */
#define LED_BLINK_FREQ_HZ   1u
#define LED_TASK_FREQ_HZ    (LED_BLINK_FREQ_HZ * 2u)

static void led_ctrl_run(void *argument)
{
    static bool led_on;

    (void)argument;
    led_on = !led_on;
    if (led_on)
    {
        ws2812_set(0, LED_BRIGHT, LED_BRIGHT, LED_BRIGHT);
    }
    else
    {
        ws2812_set(0, 0, 0, 0);
    }
    (void)ws2812_refresh();
}
/* 任务名 led_ctrl_task，2 Hz，优先级 3（数值小优先级高），栈 256 字 */
PERIOD_TASK_DEFINE(led_ctrl_task, LED_TASK_FREQ_HZ, 3, 256, led_ctrl_run, NULL);
