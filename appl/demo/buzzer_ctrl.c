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
/* shell 与周期任务共用播放状态，所有运行期播放操作都在此锁内。 */
static os_mutex_t buzzer_mutex;
static bool buzzer_ready;
static bool buzzer_ok;
static uint32_t tick_count;

static int32_t buzzer_ctrl_init(void)
{
    int32_t ret = os_mutex_init(&buzzer_mutex);
    if (ret != ERR_OK) return ret;
    buzzer_ready = true;
    next_ready_ms = bsp_tick_ms() + BUZZER_READY_INTERVAL_MS;
    ret = buzzer_play(&buzzer_music_ready, BUZZER_INIT_VOLUME);
    buzzer_ok = (ret == ERR_OK);
    tick_count = 1;
    return ret;
}
INIT_APP_STAGE2(buzzer_ctrl_init);

static void buzzer_run(void *argument)
{
    (void)argument;
    if (!buzzer_ready || os_mutex_lock(&buzzer_mutex, OS_NO_WAIT) != ERR_OK) return;
    if (!buzzer_ok)
    {
        os_mutex_unlock(&buzzer_mutex);
        return;
    }
    if (buzzer_tick() != ERR_OK)
    {
        buzzer_ok = false;
        os_mutex_unlock(&buzzer_mutex);
        return;
    }
    uint32_t now = bsp_tick_ms();
    if (!buzzer_is_playing() && (int32_t)(now - next_ready_ms) >= 0)
    {
        buzzer_ok = (buzzer_play(&buzzer_music_startup, BUZZER_RUN_VOLUME) == ERR_OK);
        next_ready_ms = now + BUZZER_READY_INTERVAL_MS;
    }
    tick_count ++;
    os_mutex_unlock(&buzzer_mutex);
}
PERIOD_TASK_DEFINE(buzzer_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_LOWEST, OS_STACK_SMALL, buzzer_run, NULL);

#ifdef CONFIG_COMPONENT_SHELL
#include <string.h>
#include "component/shell/shell_cmd.h"

/* 不带参数时列出曲库。you 是 robomaster_you 的别名。 */
static int shell_buzzer_play(int argc, char *argv[])
{
    if (argc != 2)
    {
        shell_out("usage: buzzer_play <song>\r\nsongs:");
        for (uint16_t i = 0u; i < buzzer_song_count; i++)
        {
            shell_out(" %s", buzzer_songs[i].name);
        }
        shell_out("\r\nalias: you = robomaster_you\r\n");
        return argc == 1 ? ERR_OK : ERR_INVAL;
    }

    const char *name = strcmp(argv[1], "you") == 0 ? "robomaster_you" : argv[1];
    const buzzer_song_t *song = NULL;
    for (uint16_t i = 0u; i < buzzer_song_count; i++)
    {
        if (strcmp(name, buzzer_songs[i].name) == 0)
        {
            song = &buzzer_songs[i];
            break;
        }
    }
    if (song == NULL)
    {
        shell_out("unknown song: %s; run buzzer_play to list songs\r\n", argv[1]);
        return ERR_INVAL;
    }
    if (!buzzer_ready) return ERR_NODEV;
    if (os_mutex_lock(&buzzer_mutex, OS_NO_WAIT) != ERR_OK)
    {
        shell_out("buzzer busy; retry\r\n");
        return ERR_BUSY;
    }
    int32_t ret = buzzer_ok ? buzzer_play(song, BUZZER_RUN_VOLUME) : ERR_NODEV;
    if (ret == ERR_OK)
    {
        next_ready_ms = bsp_tick_ms() + BUZZER_READY_INTERVAL_MS;
    }
    else if (ret != ERR_BUSY && ret != ERR_INVAL)
    {
        buzzer_ok = false;
    }
    os_mutex_unlock(&buzzer_mutex);

    if (ret == ERR_OK) shell_out("playing: %s\r\n", song->name);
    else if (ret == ERR_BUSY) shell_out("buzzer is already playing\r\n");
    else shell_out("buzzer play failed: %ld\r\n", (long)ret);
    return ret;
}
SHELL_CMD(buzzer_play, shell_buzzer_play, play_song_by_name_or_list_songs)
#endif
