#include "cdd/vofa/vofa.h"
#include <string.h>
#include "bsp/uart/bsp_uart.h"
#include "bsp/include/bsp_ids.h"
#include "component/common/err.h"

int32_t vofa_send(const float *data, uint8_t count)
{
    uint8_t frame[VOFA_CH_MAX * sizeof(float) + 4u];
    uint32_t length;
    static const uint8_t tail[4] = { 0x00u, 0x00u, 0x80u, 0x7fu };

    if (data == NULL || count == 0u || count > VOFA_CH_MAX)
    {
        return ERR_INVAL;
    }
    length = (uint32_t)count * sizeof(float);
    memcpy(frame, data, length);
    memcpy(frame + length, tail, sizeof tail);
    length += sizeof tail;
    if (bsp_uart_write(UART_VOFA, frame, length) != (int32_t)length)
    {
        return ERR_BUSY;
    }
    return ERR_OK;
}
