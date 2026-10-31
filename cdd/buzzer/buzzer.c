#include "cdd/buzzer/buzzer.h"
#include "bsp/pwm/bsp_pwm.h"
#include "bsp/tick/bsp_tick.h"
#include "component/common/err.h"

static const buzzer_song_t *playing_song;
static uint16_t note_index;
static uint16_t playing_volume;
static uint32_t next_note_ms;

/**********************************************************
* @ 函数名: buzzer_is_playing
* @ 函数作用: 查询是否仍有正在播放的曲目，曲内休止也视为播放中
* @ 返回数值: true 为正在播放，false 为空闲
**********************************************************/
bool buzzer_is_playing(void)
{
    return playing_song != NULL;
}

/**********************************************************
* @ 函数名: buzzer_set_volume
* @ 函数作用: 将感知音量按平方曲线映射到 0..50% PWM 占空比，再输出指定音高
* @ 参数信息: frequency_hz 音符频率，单位 Hz；0 为静音
* @ 参数信息: volume 感知音量，范围 0..1000；50 对应 0.125% 占空比
* @ 返回数值: ERR_OK 为成功，ERR_INVAL 为音量越界，其他值为 PWM 输出错误码
**********************************************************/
int32_t buzzer_set_volume(uint32_t frequency_hz, uint16_t volume)
{
    if (volume > 1000u) return ERR_INVAL;
    uint32_t duty_ppm = ((uint32_t)volume * volume + 1u) / 2u;
    return buzzer_set(frequency_hz, duty_ppm);
}

/**********************************************************
* @ 函数名: buzzer_set
* @ 函数作用: 直接设置蜂鸣器 PWM 频率与占空比，频率或占空比为 0 时静音
* @ 参数信息: frequency_hz 频率，单位 Hz
* @ 参数信息: duty_ppm 占空比，范围 0..1000000；10000 表示 1%
* @ 返回数值: ERR_OK 为成功，其他值为参数或 PWM 操作错误码
**********************************************************/
int32_t buzzer_set(uint32_t frequency_hz, uint32_t duty_ppm)
{
    if (duty_ppm > 1000000u) return ERR_INVAL;
    /* 先关闭 PWM：频率或占空比为 0 时也必须实际停输出，再返回停止结果。 */
    int32_t ret = bsp_pwm_stop(PWM_BUZZER);
    if (ret != ERR_OK || frequency_hz == 0u || duty_ppm == 0u) return ret;

    ret = bsp_pwm_set_freq(PWM_BUZZER, frequency_hz);
    if (ret == ERR_OK) ret = bsp_pwm_set_duty_ppm(PWM_BUZZER, duty_ppm);
    if (ret == ERR_OK) ret = bsp_pwm_start(PWM_BUZZER);
    return ret;
}

/**********************************************************
* @ 函数名: buzzer_play_note
* @ 函数作用: 输出当前音符，推进音符索引并记录下一次切换时间；输出失败时静音并停止播放
* @ 参数信息: now 当前开机时间，单位 ms；调用前需保证曲目有效且音符索引未越界
* @ 返回数值: ERR_OK 为成功，其他值为蜂鸣器输出错误码
**********************************************************/
static int32_t buzzer_play_note(uint32_t now)
{
    const buzzer_note_t *note = &playing_song->notes[note_index];
    int32_t ret = buzzer_set_volume(note->frequency_hz, playing_volume);
    if (ret != ERR_OK)
    {
        (void)buzzer_set(0u, 0u);
        playing_song = NULL;
        return ret;
    }
    note_index++;
    next_note_ms = now + note->duration_ms;
    return ERR_OK;
}

/**********************************************************
* @ 函数名: buzzer_play
* @ 函数作用: 启动指定曲目并立即输出首音，播放期间不接受另一首曲目
* @ 参数信息: song 曲目指针，曲谱须在整个播放期间保持有效
* @ 参数信息: volume 感知音量，范围 0..1000
* @ 返回数值: ERR_OK 为成功，ERR_INVAL 为参数无效，ERR_BUSY 为正在播放，其他值为输出错误码
**********************************************************/
int32_t buzzer_play(const buzzer_song_t *song, uint16_t volume)
{
    if (song == NULL || song->notes == NULL || song->count == 0u || volume > 1000u) return ERR_INVAL;
    if (playing_song != NULL) return ERR_BUSY;
    playing_song = song;
    playing_volume = volume;
    note_index = 0u;
    return buzzer_play_note(bsp_tick_ms());
}

/**********************************************************
* @ 函数名: buzzer_tick
* @ 函数作用: 到达切换时间时输出下一音符，曲目结束时静音；由周期任务调用
* @ 返回数值: ERR_OK 为正常运行或空闲，其他值为输出错误码；错误后停止播放
**********************************************************/
int32_t buzzer_tick(void)
{
    uint32_t now = bsp_tick_ms();
    if (playing_song == NULL || (int32_t)(now - next_note_ms) < 0) return ERR_OK;
    if (note_index == playing_song->count)
    {
        playing_song = NULL;
        return buzzer_set(0u, 0u);
    }
    return buzzer_play_note(now);
}
