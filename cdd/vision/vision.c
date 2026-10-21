#include <string.h>
#include "cdd/vision/vision.h"
#include "vision_config.h"
#include "bsp/usb/bsp_usb.h"
#include "osal/event.h"
#include "osal/osal.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/crc/crc.h"

_Static_assert(VISION_PAYLOAD_MAX <= 255u, "vision payload length is one byte");

#define VISION_RX_BLOCK_MAX   64u     /* 本板 USB 枚举为全速 CDC，一次接收最多 64 字节 */
#define VISION_HEADER_LENGTH  3u      /* 帧头 + 长度 + 命令 */
#define VISION_CRC_LENGTH     2u
#define VISION_MAX_FRAME      (VISION_HEADER_LENGTH + VISION_PAYLOAD_MAX + VISION_CRC_LENGTH)
#define VISION_STREAM_SIZE    (VISION_MAX_FRAME - 1u + VISION_RX_BLOCK_MAX)

typedef struct
{
    uint8_t command;
    uint8_t length;
    void   *cache;
    bool    seen;
} vision_slot_t;

static vision_link_t link;
static vision_slot_t packets[VISION_CMD_MAX];
static uint8_t packet_count;
static uint8_t stream[VISION_STREAM_SIZE];
static uint32_t stream_count;

static vision_slot_t *find_packet(uint8_t command)
{
    for (uint8_t index = 0; index < packet_count; index++)
    {
        if (packets[index].command == command)
        {
            return &packets[index];
        }
    }
    return NULL;
}

int32_t vision_bind(const vision_packet_t *table, uint8_t count)
{
    if (table == NULL || count == 0u || count > VISION_CMD_MAX)
    {
        return ERR_INVAL;
    }
    for (uint8_t index = 0; index < count; index++)
    {
        if (table[index].length > VISION_PAYLOAD_MAX || (table[index].length != 0u && table[index].cache == NULL))
        {
            return ERR_INVAL;
        }
        if (find_packet(table[index].command) != NULL)
        {
            return ERR_INVAL;
        }
        packets[index].command = table[index].command;
        packets[index].length = table[index].length;
        packets[index].cache = table[index].cache;
        packets[index].seen = false;
        packet_count++;
    }
    return ERR_OK;
}

int32_t vision_write(uint8_t command, const void *payload, uint8_t length)
{
    vision_slot_t *slot = find_packet(command);

    if (slot == NULL || length != slot->length || (payload == NULL && length != 0u))
    {
        return ERR_INVAL;
    }
    if (!bsp_usb_connected())
    {
        return ERR_NODEV;
    }

    uint8_t frame[VISION_MAX_FRAME];
    uint32_t frame_length = VISION_HEADER_LENGTH + (uint32_t)length + VISION_CRC_LENGTH;

    frame[0] = VISION_SOF;
    frame[1] = length;
    frame[2] = command;
    if (length != 0u)
    {
        memcpy(frame + VISION_HEADER_LENGTH, payload, length);
    }
    crc16_append(frame, frame_length);

    uint32_t irq_key = os_irq_save();

    if (length != 0u && slot->cache != NULL)
    {
        memcpy(slot->cache, payload, length);
    }
    os_irq_restore(irq_key);

    int32_t queued = bsp_usb_write(frame, frame_length);

    return (queued == (int32_t)frame_length) ? ERR_OK : ERR_BUSY;
}

bool vision_read(uint8_t command, void *payload, uint8_t length)
{
    uint32_t irq_key = os_irq_save();
    vision_slot_t *slot = find_packet(command);
    bool ok = slot != NULL && slot->seen && length == slot->length &&
              (length == 0u || (payload != NULL && slot->cache != NULL));

    if (ok && length != 0u)
    {
        memcpy(payload, slot->cache, length);
    }
    os_irq_restore(irq_key);
    return ok;
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
    vision_slot_t *slot = find_packet(command);
    uint32_t irq_key = os_irq_save();

    link.online = true;
    if (slot == NULL || length != slot->length)
    {
        link.bad_frames++;
        os_irq_restore(irq_key);
        return;
    }
    if (length != 0u && slot->cache != NULL && payload != NULL)
    {
        memcpy(slot->cache, payload, length);
    }
    slot->seen = true;
    link.frames++;
    os_irq_restore(irq_key);
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

#define VISION_DEFINE(name, id, type)   static type vision_##name;
VISION_PACKET_TABLE(VISION_DEFINE)
#undef VISION_DEFINE

#define VISION_ROW(name, id, type) \
    { (id), (uint8_t)sizeof(vision_##name), &vision_##name },

static const vision_packet_t vision_packets[] =
{
    VISION_PACKET_TABLE(VISION_ROW)
};
#undef VISION_ROW

static int32_t vision_init(void)
{
    int32_t bind_result = vision_bind(vision_packets, (uint8_t)(sizeof vision_packets / sizeof vision_packets[0]));

    if (bind_result != ERR_OK)
    {
        return bind_result;
    }
    bsp_usb_bind_event(&vision_task);
    return ERR_OK;
}
INIT_CDD_STAGE1(vision_init);
