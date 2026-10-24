/* 底盘板：板载 WS2812，1 Hz 亮灭（亮 500 ms、灭 500 ms）。 */
#include "osal/period.h"
#include "cdd/led/ws2812/ws2812.h"

#define LED_BRIGHT          32u

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
PERIOD_TASK_DEFINE(led_ctrl_task, OS_FREQ_2HZ, OS_PRIORITY_LOWEST, OS_STACK_SMALL, led_ctrl_run, NULL);

#ifdef CONFIG_COMPONENT_SHELL
#include <string.h>
#include "component/shell/shell_cmd.h"

static int shell_led(int argc, char *argv[])
{
    if (argc == 2 && strcmp(argv[1], "on") == 0)
    {
        ws2812_set(0, LED_BRIGHT, LED_BRIGHT, LED_BRIGHT);
    }
    else if (argc == 2 && strcmp(argv[1], "off") == 0)
    {
        ws2812_set(0, 0, 0, 0);
    }
    else
    {
        shell_out("usage: led [on|off]\r\n");
        return -1;
    }
    (void)ws2812_refresh();
    return 0;
}
SHELL_CMD(led, shell_led, turn_board_led_on_or_off)
#endif
