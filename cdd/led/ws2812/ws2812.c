#include <string.h>
#include <math.h>
#include "cdd/led/ws2812/ws2812.h"
#include "bsp/spi/bsp_spi.h"
#include "bsp/tick/bsp_tick.h"

/* SPI6 内核时钟 24 MHz(HSE)，分频 4 -> 6 MHz，每个 SPI 位 166.7 ns。
 * 一个 WS2812 位用一个 SPI 字节表示（1.33 us）：
 *   '0' = 0b11000000 -> 高 0.33 us  低 1.0 us
 *   '1' = 0b11111000 -> 高 0.83 us  低 0.5 us
 * 都落在 WS2812B 的时序容差内。数据后补 40 个 0x00 字节（>= 53 us 低电平）作为复位。 */
#define BIT0     0xC0u
#define BIT1     0xF8u
#define RESET_BYTES  40u
#define BREATH_PERIOD_MS  1000u

/*
| 颜色 | 配置值 |
|---|---|
| 红 | 0xFF0000u |
| 绿 | 0x00FF00u |
| 蓝 | 0x0000FFu |
| 黄 | 0xFFFF00u |
| 青 | 0x00FFFFu |
| 紫 | 0x8000FFu |
| 白 | 0xFFFFFFu |
| 灭 | 0x000000u |
*/

/* sRGB 解码为线性通道强度；标准分段曲线，取最近的 8 位整数。 */
static const uint8_t srgb_linear8[256] = {
    0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
    4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7,
    8, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13,
    13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 17, 18, 18, 19, 19, 20,
    20, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 27, 27, 28, 29, 29,
    30, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37, 37, 38, 39, 40, 41,
    41, 42, 43, 44, 45, 45, 46, 47, 48, 49, 50, 51, 51, 52, 53, 54,
    55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
    71, 72, 73, 74, 76, 77, 78, 79, 80, 81, 82, 84, 85, 86, 87, 88,
    90, 91, 92, 93, 95, 96, 97, 99, 100, 101, 103, 104, 105, 107, 108, 109,
    111, 112, 114, 115, 116, 118, 119, 121, 122, 124, 125, 127, 128, 130, 131, 133,
    134, 136, 138, 139, 141, 142, 144, 146, 147, 149, 151, 152, 154, 156, 157, 159,
    161, 163, 164, 166, 168, 170, 171, 173, 175, 177, 179, 181, 183, 184, 186, 188,
    190, 192, 194, 196, 198, 200, 202, 204, 206, 208, 210, 212, 214, 216, 218, 220,
    222, 224, 226, 229, 231, 233, 235, 237, 239, 242, 244, 246, 248, 250, 253, 255,
};

/* 感知亮度映射到线性输出强度，Gamma=2.8；只作用于亮度。 */
static const uint8_t gamma8[256] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
	2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
	5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10,
	10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
	17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
	25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
	37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
	51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
	69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
	90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
	115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
	144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
	177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
	215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255};

static struct
{
    uint32_t color;
    uint8_t min_brightness;
    uint8_t max_brightness;
} breath;

static uint8_t pixels[WS2812_NUM][3];                      /* GRB 顺序 */
static uint8_t frame_buffer[WS2812_NUM * 24u + RESET_BYTES];

/**********************************************************
* @ 函数名: ws2812_set_linear
* @ 函数作用: 将线性 RGB 强度按 GRB 顺序写入灯珠缓存，不进行 Gamma 转换或发送
* @ 参数信息: index 灯珠索引，范围 0..WS2812_NUM-1；越界时忽略
* @ 参数信息: red、green、blue 线性通道强度，范围 0..255
**********************************************************/
static void ws2812_set_linear(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (index >= WS2812_NUM)
    {
        return;
    }
    pixels[index][0] = green;
    pixels[index][1] = red;
    pixels[index][2] = blue;
}

/**********************************************************
* @ 函数名: ws2812_set
* @ 函数作用: 将 sRGB 颜色转换为线性通道强度后写入缓存，调用 refresh 后生效
* @ 参数信息: index 灯珠索引，范围 0..WS2812_NUM-1；越界时忽略
* @ 参数信息: red、green、blue sRGB 通道值，范围 0..255
**********************************************************/
void ws2812_set(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    ws2812_set_linear(index, srgb_linear8[red], srgb_linear8[green], srgb_linear8[blue]);
}

/**********************************************************
* @ 函数名: ws2812_refresh
* @ 函数作用: 将灯珠缓存编码为 WS2812 时序数据，追加复位低电平并通过 SPI6 发送
* @ 返回数值: ERR_OK 为发送成功，其他值为 SPI 传输错误码
**********************************************************/
int32_t ws2812_refresh(void)
{
    uint8_t *cursor = frame_buffer;

    for (uint32_t pixel_index = 0; pixel_index < WS2812_NUM; pixel_index++)
    {
        for (uint32_t color_index = 0; color_index < 3u; color_index++)
        {
            for (int bit = 7; bit >= 0; bit--)
            {
                *cursor = ((pixels[pixel_index][color_index] >> bit) & 1u) ? BIT1 : BIT0;
                cursor++;
            }
        }
    }
    memset(cursor, 0, RESET_BYTES);
    return bsp_spi_xfer(SPI_LED, BSP_SPI_NO_CS, frame_buffer, NULL, (uint16_t)sizeof frame_buffer);
}

/**********************************************************
* @ 函数名: ws2812_breath_config
* @ 函数作用: 设置呼吸灯配置信息
* @ 参数信息: color uint32_t sRGB 颜色，格式 0xRRGGBB
* @                  | 红   | 0xFF0000u |
* @                  | 绿   | 0x00FF00u |
* @                  | 蓝   | 0x0000FFu |
* @                  | 黄   | 0xFFFF00u |
* @                  | 青   | 0x00FFFFu |
* @                  | 紫   | 0x8000FFu |
* @                  | 白   | 0xFFFFFFu |
* @                  | 灭   | 0x000000u |
* @ 参数信息: min_brightness uint8_t 最小感知亮度，范围 0..255
* @ 参数信息: max_brightness uint8_t 最大感知亮度，范围 0..255
* @ 返回数值: min > max 时返回 false 且保留原配置，成功更新返回 true
**********************************************************/
bool ws2812_breath_config(uint32_t color, uint8_t min_brightness, uint8_t max_brightness)
{
    if (min_brightness > max_brightness)
    {
        return false;
    }
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    breath.color = color & 0xFFFFFFu;
    breath.min_brightness = min_brightness;
    breath.max_brightness = max_brightness;
    __set_PRIMASK(irq_state);
    return true;
}

/**********************************************************
* @ 函数名: ws2812_breath_tick
* @ 函数作用: 根据时间计算呼吸亮度，分别转换颜色与亮度后缩放并刷新灯珠
* @ 返回数值: ERR_OK 为刷新成功，其他值为 SPI 传输错误码
**********************************************************/
int32_t ws2812_breath_tick(void)
{
    uint32_t color;
    uint8_t min_brightness;
    uint8_t max_brightness;
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    color = breath.color;
    min_brightness = breath.min_brightness;
    max_brightness = breath.max_brightness;
    __set_PRIMASK(irq_state);

    float phase = (float)(bsp_tick_ms() % BREATH_PERIOD_MS) / (float)BREATH_PERIOD_MS;
    float level = 0.5f + 0.5f * sinf(phase * 6.2831853f);
    /* 先在感知亮度上下限之间插值，再映射到线性强度。 */
    uint8_t level_index = (uint8_t)(min_brightness +
                          (max_brightness - min_brightness) * level + 0.5f);
    uint8_t brightness = gamma8[level_index];

    /* 颜色独立做 sRGB 解码，三个线性通道乘同一亮度；输出不再做 Gamma。 */
    uint8_t red = (uint8_t)(((uint16_t)srgb_linear8[(color >> 16) & 0xFFu] * brightness + 127u) / 255u);
    uint8_t green = (uint8_t)(((uint16_t)srgb_linear8[(color >> 8) & 0xFFu] * brightness + 127u) / 255u);
    uint8_t blue = (uint8_t)(((uint16_t)srgb_linear8[color & 0xFFu] * brightness + 127u) / 255u);
    ws2812_set_linear(0u, red, green, blue);
    return ws2812_refresh();
}

/**********************************************************
* gamma 校正
* 对于人类来说，无论是声音大小还是亮度大小都是受到了暗部和
* 低音部的影响的，简单来概括就是美术上的均匀和物理上的均匀
* 比较。当你看一个黑色到白色的变化过程条的时候，人类视觉认
* 为的中灰色大约是物理意义上中灰色的20%。
* 为了将人类的感受和自然界线性增长的亮度、音量分贝映射，则
* 需要使用 gamma 曲线校正
* 下面是 gamma 曲线公式
**********************************************************/
