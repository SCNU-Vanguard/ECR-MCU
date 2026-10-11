#ifndef CDD_BUZZER_H__
#define CDD_BUZZER_H__
#include "component/common/types.h"
void buzzer_on(uint32_t freq_hz);     /* 50% 占空比，指定音调 */
void buzzer_off(void);
#endif
