#ifndef CDD_BUZZER_MUSIC_H__
#define CDD_BUZZER_MUSIC_H__
#include "component/common/types.h"

typedef struct
{
    uint16_t frequency_hz; /* 0 表示休止 */
    uint16_t duration_ms;
} buzzer_note_t;

typedef struct
{
    const char *name;
    const buzzer_note_t *notes;
    uint16_t count;
} buzzer_song_t;

extern const buzzer_song_t buzzer_songs[];
extern const uint16_t buzzer_song_count; /* 自动按曲目表计算。 */
/* 提示音直接按名称引用，不依赖曲库排列顺序。 */
extern const buzzer_song_t buzzer_music_startup;
extern const buzzer_song_t buzzer_music_ready;

/* 0 为休止，1..108 为 C0..B8；A4 = 58（440 Hz）。 */
#define BUZZER_NOTE_COUNT 109u
extern const uint16_t buzzer_note_freq[BUZZER_NOTE_COUNT];
#endif
