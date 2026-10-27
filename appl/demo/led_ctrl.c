/* demo：板载单颗 WS2812 呼吸灯，周期由驱动的 BREATH_PERIOD_MS 设置。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/led/ws2812/ws2812.h"

/* 单灯默认配置；颜色为 sRGB 0xRRGGBB，感知亮度范围为 0..255。 */
#define WS2812_BREATH_COLOR       0x40E476u
#define WS2812_BREATH_MIN         0u
#define WS2812_BREATH_MAX         255u

/**********************************************************
* @ 函数名: led_ctrl_init
* @ 函数作用: 在应用初始化阶段配置呼吸灯颜色与感知亮度上下限
* @ 返回数值: ERR_OK 为配置成功，ERR_INVAL 为亮度范围无效
**********************************************************/
static int32_t led_ctrl_init(void)
{
    return ws2812_breath_config(WS2812_BREATH_COLOR,
                               WS2812_BREATH_MIN,
                               WS2812_BREATH_MAX) ? ERR_OK : ERR_INVAL;
}
/* 应用初始化阶段设置呼吸灯参数，周期任务在全部初始化完成后启动。 */
INIT_APP_STAGE1(led_ctrl_init);

/**********************************************************
* @ 函数名: led_ctrl_run
* @ 函数作用: 周期更新呼吸灯亮度，并将当前颜色发送给灯珠
* @ 参数信息: argument 周期任务参数，此处未使用
**********************************************************/
static void led_ctrl_run(void *argument)
{
    (void)argument;
    (void)ws2812_breath_tick();

    /* 从IMU获取温度，根据IMU温度作颜色变化，温度越高越红色 */
    // bool change_bitstatus = ws2812_breath_config();
}
/* 任务名 led_ctrl_task，50 Hz，最低优先级，小栈 */
PERIOD_TASK_DEFINE(led_ctrl_task, OS_FREQ_50HZ, OS_PRIORITY_LOWEST, OS_STACK_SMALL, led_ctrl_run, NULL);
