/* 单板：蜂鸣器存活提示。开机响一声，之后每 30 秒响一声。
 * 10 Hz 周期：计数 0 开，计数 1 关，计数到 300（30 s）回到 0。 */
#include "osal/period.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/buzzer/buzzer.h"

#define BUZZER_TONE_HZ          2000u
#define BUZZER_BEEP_INTERVAL_S  30u
#define BUZZER_INTERVAL_TICKS   (BUZZER_BEEP_INTERVAL_S * OS_FREQ_10HZ)

static uint32_t tick_count;

static int32_t buzzer_ctrl_init(void)
{
    buzzer_on(BUZZER_TONE_HZ);
    tick_count = 1;
    return ERR_OK;
}
INIT_APP_STAGE2(buzzer_ctrl_init);

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
PERIOD_TASK_DEFINE(buzzer_ctrl_task, OS_FREQ_10HZ, OS_PRIORITY_LOWEST, OS_STACK_SMALL, buzzer_ctrl_run, NULL);
