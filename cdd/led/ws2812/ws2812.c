#include <string.h>
#include "cdd/led/ws2812/ws2812.h"
#include "bsp/spi/bsp_spi.h"
#include "component/common/err.h"

/* SPI6 内核时钟 24 MHz(HSE)，分频 4 -> 6 MHz，每个 SPI 位 166.7 ns。
 * 一个 WS2812 位用一个 SPI 字节表示（1.33 us）：
 *   '0' = 0b11000000 -> 高 0.33 us  低 1.0 us
 *   '1' = 0b11111000 -> 高 0.83 us  低 0.5 us
 * 都落在 WS2812B 的时序容差内。数据后补 40 个 0x00 字节（>= 53 us 低电平）作为复位。 */
#define BIT0     0xC0u
#define BIT1     0xF8u
#define RESET_BYTES  40u

static uint8_t pixels[WS2812_NUM][3];                      /* GRB 顺序 */
static uint8_t frame_buffer[WS2812_NUM * 24u + RESET_BYTES];

void ws2812_set(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (index >= WS2812_NUM)
    {
        return;
    }
    pixels[index][0] = green;
    pixels[index][1] = red;
    pixels[index][2] = blue;
}

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
