/* demo：初始化播放 ready 一次，之后每 60 秒播放 startup 作为运行提示。 */
#include "osal/period.h"
#include "bsp/tick/bsp_tick.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/buzzer/buzzer.h"
#include "cdd/buzzer/buzzer_music.h"

#define BUZZER_INIT_VOLUME 1000u /* 0..1000；1000 对应 50% 占空比，50 对应 0.125%。 */
#define BUZZER_RUN_VOLUME 200u /* 0..1000；1000 对应 50% 占空比，50 对应 0.125%。 */
#define BUZZER_READY_INTERVAL_MS 60000u /* 两次运行提示音开始播放的间隔。 */
static uint32_t next_ready_ms;
static bool buzzer_ok;
static uint32_t tick_count;

/**********************************************************
* @ 函数名: buzzer_ctrl_init
* @ 函数作用: 播放开机提示音 ready，并安排首次运行提示在 60 秒后播放
* @ 返回数值: ERR_OK 为成功，其他值为启动播放失败的错误码
**********************************************************/
static int32_t buzzer_ctrl_init(void)
{
    next_ready_ms = bsp_tick_ms() + BUZZER_READY_INTERVAL_MS;
    int32_t ret = buzzer_play(&buzzer_music_ready, BUZZER_INIT_VOLUME);
    buzzer_ok = (ret == ERR_OK);
    tick_count = 1;
    return ret;
}
INIT_APP_STAGE2(buzzer_ctrl_init);

/**********************************************************
* @ 函数名: buzzer_run
* @ 函数作用: 推进曲目播放，每 60 秒播放一次 startup；输出失败后停止后续提示
* @ 参数信息: argument 周期任务参数，此处未使用
**********************************************************/
static void buzzer_run(void *argument)
{
    (void)argument;
    if (!buzzer_ok) return;
    if (buzzer_tick() != ERR_OK)
    {
        buzzer_ok = false;
        return;
    }
    uint32_t now = bsp_tick_ms();
    if (!buzzer_is_playing() && (int32_t)(now - next_ready_ms) >= 0)
    {
        buzzer_ok = (buzzer_play(&buzzer_music_startup, BUZZER_RUN_VOLUME) == ERR_OK);
        next_ready_ms = now + BUZZER_READY_INTERVAL_MS;
    }
    tick_count ++;
}
/* 全部音乐一起播放，曲间静音 500 ms，播完停止。
 * 启用此代码块、停用上面的初始化注册，并将任务回调改为 buzzer_play_all 即可试听。 */
#if 0
/**********************************************************
* @ 函数名: buzzer_play_all
* @ 函数作用: 按曲库顺序试听全部曲目，曲间静音 500 ms，全部播完后停止
* @ 参数信息: argument 周期任务参数，此处未使用
**********************************************************/
static void buzzer_play_all(void *argument)
{
    static uint16_t song_index;
    static bool song_started;
    static uint32_t next_song_ms;
    (void)argument;
    if (buzzer_tick() != ERR_OK)
    {
        song_index = buzzer_song_count;
        return;
    }
    if (buzzer_is_playing()) return;

    uint32_t now = bsp_tick_ms();
    if (song_started)
    {
        song_started = false;
        next_song_ms = now + 500u;
        return;
    }
    if (song_index >= buzzer_song_count || (int32_t)(now - next_song_ms) < 0) return;
    if (buzzer_play(&buzzer_songs[song_index], BUZZER_RUN_VOLUME) != ERR_OK)
    {
        song_index = buzzer_song_count;
        return;
    }
    song_index++;
    song_started = true;
}
#endif

PERIOD_TASK_DEFINE(buzzer_task, OS_FREQ_1000HZ, OS_PRIORITY_LOWEST, OS_STACK_SMALL, buzzer_run, NULL);
