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

static uint8_t pixel[WS2812_NUM][3];                       /* GRB 顺序 */
static uint8_t frame[WS2812_NUM * 24u + RESET_BYTES];

void ws2812_set(uint8_t idx, uint8_t r, uint8_t g, uint8_t b)
{
    if (idx >= WS2812_NUM) { return; }
    pixel[idx][0] = g; pixel[idx][1] = r; pixel[idx][2] = b;
}

int32_t ws2812_refresh(void)
{
    uint8_t *p = frame;
    for (uint32_t i = 0; i < WS2812_NUM; i++) {
        for (uint32_t c = 0; c < 3u; c++) {
            for (int bit = 7; bit >= 0; bit--) { *p++ = ((pixel[i][c] >> bit) & 1u) ? BIT1 : BIT0; }
        }
    }
    memset(p, 0, RESET_BYTES);
    return bsp_spi_xfer(SPI_LED, BSP_SPI_NO_CS, frame, NULL, (uint16_t)sizeof frame);
}
