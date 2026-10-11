/* 自测：蜂鸣器。上电后依次 1 kHz、2 kHz、3 kHz 各响 0.3 秒，然后关闭。 */
#include "osal/period.h"
#include "cdd/buzzer/buzzer.h"

static void buzzer_task(void *arg)
{
    static uint32_t n;
    static const uint16_t tone[3] = { 1000, 2000, 3000 };
    (void)arg;
    if (n < 9u) {
        if (n % 3u == 0u) { buzzer_on(tone[n / 3u]); }
        n++;
        if (n == 9u) { buzzer_off(); }
    }
}
PERIOD_TASK_DEFINE(st_buzzer, 10, 12, 384, buzzer_task, NULL);
