/* demo：蜂鸣器存活提示。
 *   - 初始化时响一声（INIT_APP 里打开，任务第一次运行时关闭，约 100 ms）
 *   - 之后每 5 秒响一声，告诉开发者程序还在跑
 * 10 Hz 周期任务：一个周期 100 ms，计数 0 时开，计数 1 时关，计数到 50（5 s）回到 0。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/buzzer/buzzer.h"

#define BUZZER_TONE_HZ          2000u
#define BUZZER_TASK_FREQ_HZ     10u
#define BUZZER_BEEP_INTERVAL_S  10u
#define BUZZER_INTERVAL_TICKS   (BUZZER_BEEP_INTERVAL_S * BUZZER_TASK_FREQ_HZ)

static uint32_t tick_count;

static int32_t buzzer_ctrl_init(void)
{
    buzzer_on(BUZZER_TONE_HZ);
    tick_count = 1;                 /* 任务第一次运行就关闭，开机响一声约 100 ms */
    return ERR_OK;
}
INIT_APP(buzzer_ctrl_init);

static void buzzer_ctrl_run(void *argument)
{
    (void)argument;
    if (tick_count == 0u)
    {
        buzzer_on(BUZZER_TONE_HZ);
    }
    else if (tick_count == 1u)
    {
        buzzer_off();
    }
    tick_count++;
    if (tick_count >= BUZZER_INTERVAL_TICKS)
    {
        tick_count = 0;
    }
}
/* 任务名 buzzer_ctrl_task，10 Hz，优先级 4（数值小优先级高），栈 256 字 */
PERIOD_TASK_DEFINE(buzzer_ctrl_task, BUZZER_TASK_FREQ_HZ, 4, 256, buzzer_ctrl_run, NULL);
