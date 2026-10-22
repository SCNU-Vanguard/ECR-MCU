#include <string.h>
#include "cdd/host/host.h"
#include "host_config.h"
#include "bsp/usb/bsp_usb.h"
#include "osal/event.h"
#include "osal/osal.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/crc/crc.h"

_Static_assert(HOST_PAYLOAD_MAX <= 255u, "host payload length is one byte");

#define HOST_RX_BLOCK_MAX   64u     /* 本板 USB 枚举为全速 CDC，一次接收最多 64 字节 */
#define HOST_HEADER_LENGTH  3u      /* 帧头 + 长度 + 命令 */
#define HOST_CRC_LENGTH     2u
#define HOST_MAX_FRAME      (HOST_HEADER_LENGTH + HOST_PAYLOAD_MAX + HOST_CRC_LENGTH)
#define HOST_STREAM_SIZE    (HOST_MAX_FRAME - 1u + HOST_RX_BLOCK_MAX)

typedef struct
{
    uint8_t command;
    uint8_t direction;
    uint8_t length;
    void   *cache;
    bool    seen;
} host_slot_t;

static host_link_t link;
static host_slot_t packets[HOST_CMD_MAX];
static uint8_t packet_count;
static uint8_t stream[HOST_STREAM_SIZE];
static uint32_t stream_count;

static host_slot_t *find_packet(uint8_t command)
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

int32_t host_bind(const host_packet_t *table, uint8_t count)
{
    if (table == NULL || count == 0u || count > HOST_CMD_MAX)
    {
        return ERR_INVAL;
    }
    for (uint8_t index = 0; index < count; index++)
    {
        if ((table[index].direction != HOST_SEND && table[index].direction != HOST_RECV) ||
            table[index].length > HOST_PAYLOAD_MAX ||
            (table[index].length != 0u && table[index].cache == NULL))
        {
            return ERR_INVAL;
        }
        if (find_packet(table[index].command) != NULL)
        {
            return ERR_INVAL;
        }
        packets[index].command = table[index].command;
        packets[index].direction = table[index].direction;
        packets[index].length = table[index].length;
        packets[index].cache = table[index].cache;
        packets[index].seen = false;
        packet_count++;
    }
    return ERR_OK;
}

static int32_t host_send(host_slot_t *slot, const void *payload, uint8_t length)
{
    if (!bsp_usb_connected())
    {
        return ERR_NODEV;
    }

    uint8_t frame[HOST_MAX_FRAME];
    uint32_t frame_length = HOST_HEADER_LENGTH + (uint32_t)length + HOST_CRC_LENGTH;

    frame[0] = HOST_SOF;
    frame[1] = length;
    frame[2] = slot->command;
    if (length != 0u)
    {
        memcpy(frame + HOST_HEADER_LENGTH, payload, length);
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

static int32_t host_recv(host_slot_t *slot, void *payload, uint8_t length)
{
    uint32_t irq_key = os_irq_save();
    bool ok = slot->seen && (length == 0u || (payload != NULL && slot->cache != NULL));

    if (ok && length != 0u)
    {
        memcpy(payload, slot->cache, length);
    }
    os_irq_restore(irq_key);
    return ok ? ERR_OK : ERR_TIMEOUT;
}

int32_t host_transfer(uint8_t command, void *payload, uint8_t length)
{
    host_slot_t *slot = find_packet(command);

    if (slot == NULL || length != slot->length || (payload == NULL && length != 0u))
    {
        return ERR_INVAL;
    }
    if (slot->direction == HOST_SEND)
    {
        return host_send(slot, payload, length);
    }
    if (slot->direction == HOST_RECV)
    {
        return host_recv(slot, payload, length);
    }
    return ERR_INVAL;
}

void host_get_link(host_link_t *out)
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
    host_slot_t *slot = find_packet(command);
    uint32_t irq_key = os_irq_save();

    link.online = true;
    if (slot == NULL || slot->direction != HOST_RECV || length != slot->length)
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
        if (stream[0] != HOST_SOF)
        {
            uint32_t skip = 1u;

            while (skip < stream_count && stream[skip] != HOST_SOF)
            {
                skip++;
            }
            drop_bytes(skip);
            continue;
        }
        if (stream_count < HOST_HEADER_LENGTH)
        {
            return;
        }

        uint8_t payload_length = stream[1];

        if (payload_length > HOST_PAYLOAD_MAX)
        {
            note_bad_frame();
            drop_bytes(1u);
            continue;
        }

        uint32_t frame_length = HOST_HEADER_LENGTH + (uint32_t)payload_length + HOST_CRC_LENGTH;

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
        accept_frame(stream[2], stream + HOST_HEADER_LENGTH, payload_length);
        drop_bytes(frame_length);
    }
}

static void host_run(void *argument, const uint8_t *data, uint16_t len)
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
    if (len > HOST_STREAM_SIZE - stream_count)
    {
        stream_count = 0u;
        note_bad_frame();
    }
    memcpy(stream + stream_count, data, len);
    stream_count += len;
    parse_stream();
}
EVENT_TASK_DEFINE(host_task, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, EVENT_QUEUE, HOST_RX_BLOCK_MAX, OS_QUEUE_LEN_8,
                  OS_TIMEOUT_200MS, host_run, NULL);

#define HOST_DEFINE(name, id, direction, type)   static type host_##name;
HOST_PACKET_TABLE(HOST_DEFINE)
#undef HOST_DEFINE

#define HOST_ROW(name, id, direction, type) \
    { (id), (direction), (uint8_t)sizeof(host_##name), &host_##name },

static const host_packet_t host_packets[] =
{
    HOST_PACKET_TABLE(HOST_ROW)
};
#undef HOST_ROW

static int32_t host_init(void)
{
    int32_t bind_result = host_bind(host_packets, (uint8_t)(sizeof host_packets / sizeof host_packets[0]));

    if (bind_result != ERR_OK)
    {
        return bind_result;
    }
    bsp_usb_bind_event(&host_task);
    return ERR_OK;
}
INIT_CDD_STAGE1(host_init);
