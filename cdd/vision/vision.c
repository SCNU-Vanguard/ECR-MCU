#include <string.h>
#include "cdd/vision/vision.h"
#include "bsp/usb/bsp_usb.h"
#include "osal/event.h"
#include "osal/osal.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/crc/crc.h"

#define VISION_RX_BLOCK_MAX   64u     /* 本板 USB 枚举为全速 CDC，一次接收最多 64 字节 */
#define VISION_HEADER_LENGTH  3u      /* 帧头 + 长度 + 命令 */
#define VISION_CRC_LENGTH     2u
#define VISION_MAX_FRAME      (VISION_HEADER_LENGTH + VISION_PAYLOAD_MAX + VISION_CRC_LENGTH)
#define VISION_STREAM_SIZE    (VISION_MAX_FRAME - 1u + VISION_RX_BLOCK_MAX)

static vision_link_t link;
static vision_rx_fn rx_fn;
static void *rx_argument;
static uint8_t stream[VISION_STREAM_SIZE];
static uint32_t stream_count;

int32_t vision_send(uint8_t command, const void *payload, uint8_t length)
{
    uint8_t frame[VISION_MAX_FRAME];

    if ((payload == NULL && length != 0u) || length > VISION_PAYLOAD_MAX)
    {
        return ERR_INVAL;
    }
    if (!bsp_usb_connected())
    {
        return ERR_NODEV;
    }

    uint32_t frame_length = VISION_HEADER_LENGTH + (uint32_t)length + VISION_CRC_LENGTH;

    frame[0] = VISION_SOF;
    frame[1] = length;
    frame[2] = command;
    if (length != 0u)
    {
        memcpy(frame + VISION_HEADER_LENGTH, payload, length);
    }
    crc16_append(frame, frame_length);

    int32_t queued = bsp_usb_write(frame, frame_length);

    return (queued == (int32_t)frame_length) ? ERR_OK : ERR_BUSY;
}

void vision_bind_rx(vision_rx_fn fn, void *argument)
{
    uint32_t irq_key = os_irq_save();

    rx_fn = fn;
    rx_argument = argument;
    os_irq_restore(irq_key);
}

void vision_get_link(vision_link_t *out)
{
    uint32_t irq_key = os_irq_save();

    *out = link;
    os_irq_restore(irq_key);
}

static void note_bad_frame(void)
{
    uint32_t irq_key = os_irq_save();

    link.bad_frames++;
    os_irq_restore(irq_key);
}

static void drop_bytes(uint32_t count)
{
    stream_count -= count;
    memmove(stream, stream + count, stream_count);
}

static void accept_frame(uint8_t command, const uint8_t *payload, uint8_t length)
{
    vision_rx_fn fn;
    void *argument;

    uint32_t irq_key = os_irq_save();

    link.online = true;
    link.frames++;
    fn = rx_fn;
    argument = rx_argument;
    os_irq_restore(irq_key);
    if (fn != NULL)
    {
        fn(command, payload, length, argument);
    }
}

/* 帧头对上之后：长度不能超过载荷上限，收齐后再验 CRC。失败只丢 1 字节，避免假 0xA5 吃掉后面的真帧 */
static void parse_stream(void)
{
    while (stream_count > 0u)
    {
        if (stream[0] != VISION_SOF)
        {
            uint32_t skip = 1u;

            while (skip < stream_count && stream[skip] != VISION_SOF)
            {
                skip++;
            }
            drop_bytes(skip);
            continue;
        }
        if (stream_count < VISION_HEADER_LENGTH)
        {
            return;
        }

        uint8_t payload_length = stream[1];

        if (payload_length > VISION_PAYLOAD_MAX)
        {
            note_bad_frame();
            drop_bytes(1u);
            continue;
        }

        uint32_t frame_length = VISION_HEADER_LENGTH + (uint32_t)payload_length + VISION_CRC_LENGTH;

        if (stream_count < frame_length)
        {
            return;
        }
        if (!crc16_verify(stream, frame_length))
        {
            note_bad_frame();
            drop_bytes(1u);
            continue;
        }
        accept_frame(stream[2], stream + VISION_HEADER_LENGTH, payload_length);
        drop_bytes(frame_length);
    }
}

static void vision_run(void *argument, const uint8_t *data, uint16_t len)
{
    (void)argument;
    if (data == NULL)
    {
        uint32_t irq_key = os_irq_save();

        link.online = false;
        os_irq_restore(irq_key);
        stream_count = 0u;
        return;
    }
    if (len > VISION_STREAM_SIZE - stream_count)
    {
        stream_count = 0u;
        note_bad_frame();
    }
    memcpy(stream + stream_count, data, len);
    stream_count += len;
    parse_stream();
}
EVENT_TASK_DEFINE(vision_task, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, EVENT_QUEUE, VISION_RX_BLOCK_MAX, OS_QUEUE_LEN_8,
                  OS_TIMEOUT_200MS, vision_run, NULL);

static int32_t vision_init(void)
{
    bsp_usb_bind_event(&vision_task);
    return ERR_OK;
}
INIT_CDD_STAGE1(vision_init);
