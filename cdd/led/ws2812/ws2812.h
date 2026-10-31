#ifndef CDD_WS2812_H__
#define CDD_WS2812_H__
#include "component/common/types.h"

#ifndef WS2812_NUM
#define WS2812_NUM  1               /* 灯珠个数：随硬件修改 */
#endif

void    ws2812_set(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);   /* sRGB 输入，转为线性输出后只改缓存 */
int32_t ws2812_refresh(void);                                                  /* 通过 SPI6 刷出去 */
/* 单灯呼吸：应用层在启动周期调用前配置；color 为 sRGB 0xRRGGBB，
 * 亮度上下限为感知亮度 0..255，Gamma=2.8；min > max 时返回 false。未配置时默认熄灭。 */
bool    ws2812_breath_config(uint32_t color, uint8_t min_brightness, uint8_t max_brightness);
int32_t ws2812_breath_tick(void);                                              /* 周期调用，推荐 50 Hz */
#endif
