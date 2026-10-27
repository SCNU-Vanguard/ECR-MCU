#ifndef CDD_BUZZER_H__
#define CDD_BUZZER_H__
#include "component/common/types.h"
#include "cdd/buzzer/buzzer_music.h"

/* 设置音调和占空比（ppm，10000=1%）；频率或占空比为 0 时静音。
 * 直接输出接口，不与曲目播放混用。 */
int32_t buzzer_set(uint32_t frequency_hz, uint32_t duty_ppm);
/* 感知音量 0..1000，平方曲线近似映射到 0..50% 占空比；0 为静音。
 * 例如音量 50 -> 1250 ppm（0.125%）；不做不同音高间的响度校准。 */
int32_t buzzer_set_volume(uint32_t frequency_hz, uint16_t volume);

/* 启动一首曲目并立即输出首音；曲谱需在播放期间有效。
 * 单任务调用；播放中返回 ERR_BUSY，音量范围 0..1000。 */
int32_t buzzer_play(const buzzer_song_t *song, uint16_t volume);
/* 周期调用以切换音符，播完自动静音；输出失败时停止播放并返回错误。 */
int32_t buzzer_tick(void);
bool    buzzer_is_playing(void);
#endif
