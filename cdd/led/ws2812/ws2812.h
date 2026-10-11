#ifndef CDD_WS2812_H__
#define CDD_WS2812_H__
#include "component/common/types.h"

#ifndef WS2812_NUM
#define WS2812_NUM  1               /* 灯珠个数：随硬件修改 */
#endif

void    ws2812_set(uint8_t idx, uint8_t r, uint8_t g, uint8_t b);   /* 只改缓存 */
int32_t ws2812_refresh(void);                                       /* 通过 SPI6 刷出去 */
#endif
