#include <string.h>
#include "cdd/host/host.h"
#include "host_config.h"
#include "bsp/usb/bsp_usb.h"
#include "osal/event.h"
#include "osal/period.h"
#include "osal/osal.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/crc/crc.h"

#ifndef HOST_TX_HZ
#error host: set HOST_TX_HZ in host_config.h
#endif

#define HOST_SOF            0xA5u
#define HOST_CMD_MAX        8u
#define HOST_RX_BLOCK_MAX   64u     /* 本板 USB 枚举为全速 CDC，一次接收最多 64 字节 */
#define HOST_RX_TIMEOUT     OS_TIMEOUT_200MS
#define HOST_HEADER_LENGTH  3u      /* 帧头 + 长度 + 命令 */
#define HOST_CRC_LENGTH     2u
#define HOST_CRC16_POLY     0x1021u     /* 正向多项式，低位先入 */
#define HOST_CRC16_INIT     0xFFFFu
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

typedef struct
{
    uint8_t command;
    uint8_t direction;
    uint8_t length;
    void   *cache;
} host_packet_t;

static host_link_t link;
static host_slot_t packets[HOST_CMD_MAX];
static uint8_t packet_count;
static uint8_t stream[HOST_STREAM_SIZE];
static uint32_t stream_count;
static uint32_t last_ok_ms;
static bool rx_seen;

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

static int32_t host_bind(const host_packet_t *table, uint8_t count)
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
        for (uint8_t previous = 0; previous < index; previous++)
        {
            if (table[previous].command == table[index].command)
            {
                return ERR_INVAL;
            }
        }
    }
    for (uint8_t index = 0; index < count; index++)
    {
        packets[index].command = table[index].command;
        packets[index].direction = table[index].direction;
        packets[index].length = table[index].length;
        packets[index].cache = table[index].cache;
        packets[index].seen = false;
    }
    packet_count = count;
    return ERR_OK;
}

static int32_t write_frame(uint8_t command, const void *payload, uint8_t length)
{
    if (!bsp_usb_connected())
    {
        return ERR_NODEV;
    }

    uint8_t frame[HOST_MAX_FRAME];
    uint32_t frame_length = HOST_HEADER_LENGTH + (uint32_t)length + HOST_CRC_LENGTH;

    frame[0] = HOST_SOF;
    frame[1] = length;
    frame[2] = command;
    if (length != 0u)
    {
        memcpy(frame + HOST_HEADER_LENGTH, payload, length);
    }
    uint16_t crc = crc16_calculate(frame, frame_length - HOST_CRC_LENGTH, HOST_CRC16_POLY, HOST_CRC16_INIT);

    frame[frame_length - 2u] = (uint8_t)(crc & 0xFFu);
    frame[frame_length - 1u] = (uint8_t)(crc >> 8);

    /* 整帧原子入队：空间不够就一个字节都不写，避免队列里留下半帧 */
    if (bsp_usb_tx_free() < frame_length)
    {
        return ERR_BUSY;
    }
    if (bsp_usb_write(frame, frame_length) != (int32_t)frame_length)
    {
        return ERR_BUSY;
    }
    return ERR_OK;
}

void host_store(uint8_t command, const void *payload, uint8_t length)
{
    host_slot_t *slot = find_packet(command);

    if (slot == NULL || slot->direction != HOST_SEND || length != slot->length ||
        (payload == NULL && length != 0u))
    {
        return;
    }

    uint32_t irq_key = os_irq_save();

    if (length != 0u)
    {
        memcpy(slot->cache, payload, length);
    }
    slot->seen = true;
    os_irq_restore(irq_key);
}

bool host_load(uint8_t command, void *payload, uint8_t length)
{
    host_slot_t *slot = find_packet(command);
    bool ok;

    if (slot == NULL || slot->direction != HOST_RECV || length != slot->length ||
        (payload == NULL && length != 0u))
    {
        return false;
    }

    uint32_t irq_key = os_irq_save();

    ok = link.online && slot->seen;
    if (ok && length != 0u)
    {
        memcpy(payload, slot->cache, length);
    }
    os_irq_restore(irq_key);
    return ok;
}

void host_get_link(host_link_t *out)
{
    if (out == NULL)
    {
        return;
    }

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
    uint32_t now = os_time_ms();
    uint32_t irq_key = os_irq_save();

    if (slot == NULL || slot->direction != HOST_RECV || length != slot->length)
    {
        link.bad_frames++;      /* 命令未注册/方向不对/长度不符：不刷新在线状态 */
        os_irq_restore(irq_key);
        return;
    }
    link.online = true;
    if (length != 0u && slot->cache != NULL && payload != NULL)
    {
        memcpy(slot->cache, payload, length);
    }
    slot->seen = true;
    link.frames++;
    rx_seen = true;
    last_ok_ms = now;
    os_irq_restore(irq_key);
}

static void host_age(void)
{
    uint32_t now = os_time_ms();
    uint32_t irq_key = os_irq_save();

    if (!rx_seen || !link.online || (now - last_ok_ms) < HOST_RX_TIMEOUT)
    {
        os_irq_restore(irq_key);
        return;
    }
    link.online = false;
    link.timeouts++;
    for (uint8_t index = 0; index < packet_count; index++)
    {
        if (packets[index].direction == HOST_RECV)
        {
            packets[index].seen = false;
        }
    }
    os_irq_restore(irq_key);
    stream_count = 0u;
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
        if (!crc16_verify(stream, frame_length, HOST_CRC16_POLY, HOST_CRC16_INIT))
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
    if (data != NULL)
    {
        if (len > HOST_STREAM_SIZE - stream_count)
        {
            stream_count = 0u;
            note_bad_frame();
        }
        memcpy(stream + stream_count, data, len);
        stream_count += len;
        parse_stream();
    }
    host_age();
}
EVENT_TASK_DEFINE(host_rx_task, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, EVENT_QUEUE, HOST_RX_BLOCK_MAX, OS_QUEUE_LEN_8,
                  HOST_RX_TIMEOUT, host_run, NULL);

static void host_tx_run(void *argument)
{
    (void)argument;
    for (uint8_t index = 0; index < packet_count; index++)
    {
        host_slot_t *slot = &packets[index];
        uint8_t payload[HOST_PAYLOAD_MAX];
        uint8_t command;
        uint8_t length;
        bool ready;
        uint32_t irq_key = os_irq_save();

        ready = slot->direction == HOST_SEND && slot->seen;
        command = slot->command;
        length = slot->length;
        if (ready && length != 0u && slot->cache != NULL)
        {
            memcpy(payload, slot->cache, length);
        }
        os_irq_restore(irq_key);
        if (!ready)
        {
            continue;
        }

        int32_t send_ret = write_frame(command, payload, length);

        if (send_ret == ERR_NODEV)
        {
            return;
        }
        irq_key = os_irq_save();
        if (send_ret == ERR_OK)
        {
            link.tx_frames++;
        }
        else
        {
            link.tx_fail++;
        }
        os_irq_restore(irq_key);
    }
}
PERIOD_TASK_DEFINE(host_tx_task, HOST_TX_HZ, OS_PRIORITY_LOW, OS_STACK_SMALL, host_tx_run, NULL);

#define HOST_DEFINE(id, direction, type)   static type host_##id;
HOST_PACKET_TABLE(HOST_DEFINE)
#undef HOST_DEFINE

#define HOST_ROW(id, direction, type) \
    { (id), (direction), (uint8_t)sizeof(host_##id), &host_##id },

static const host_packet_t host_packets[] =
{
    HOST_PACKET_TABLE(HOST_ROW)
};
#undef HOST_ROW

static int32_t host_init(void)
{
    uint8_t count = (uint8_t)(sizeof host_packets / sizeof host_packets[0]);

    if (count == 0u)
    {
        return ERR_OK;
    }

    int32_t bind_result = host_bind(host_packets, count);

    if (bind_result != ERR_OK)
    {
        return bind_result;
    }
    bsp_usb_bind_event(&host_rx_task);
    return ERR_OK;
}
INIT_CDD_STAGE1(host_init);

#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"

static int shell_host(int argc, char *argv[])
{
    host_link_t link;

    (void)argc;
    (void)argv;
    host_get_link(&link);
    shell_out("host: online=%d frames=%lu bad=%lu\r\n", (int)link.online,
              (unsigned long)link.frames, (unsigned long)link.bad_frames);
    shell_out("  tx=%lu tx_fail=%lu\r\n", (unsigned long)link.tx_frames, (unsigned long)link.tx_fail);
    return 0;
}
SHELL_CMD(host, shell_host, show_host_link)
#endif
