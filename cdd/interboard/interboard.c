#include <string.h>
#include "cdd/interboard/interboard.h"
#include "interboard_config.h"
#include "bsp/uart/bsp_uart.h"
#include "osal/event.h"
#include "osal/osal.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/crc/crc.h"

#if defined(CONFIG_CDD_INTERBOARD_MASTER) && defined(CONFIG_CDD_INTERBOARD_SLAVE)
#error interboard: set only MASTER or only SLAVE
#endif
#if !defined(CONFIG_CDD_INTERBOARD_MASTER) && !defined(CONFIG_CDD_INTERBOARD_SLAVE)
#error interboard: set CONFIG_CDD_INTERBOARD_MASTER or CONFIG_CDD_INTERBOARD_SLAVE
#endif

_Static_assert(INTERBOARD_PAYLOAD_MAX <= 255u, "interboard payload length is one byte");

#define INTERBOARD_SOF            0xA5u
#define INTERBOARD_RX_BLOCK_MAX   128u    /* 与 board.c 里 interboard 接收缓冲一致 */
#define INTERBOARD_HEADER_LENGTH  4u      /* 帧头 + 长度 + 命令 + 序号 */
#define INTERBOARD_CRC_LENGTH     2u
#define INTERBOARD_MAX_FRAME      (INTERBOARD_HEADER_LENGTH + INTERBOARD_PAYLOAD_MAX + INTERBOARD_CRC_LENGTH)
#define INTERBOARD_STREAM_SIZE    (INTERBOARD_MAX_FRAME - 1u + INTERBOARD_RX_BLOCK_MAX)
#define INTERBOARD_REPLY_MS       10u
#define INTERBOARD_OFFLINE_GAP_MS 50u
#define INTERBOARD_OFFLINE_STREAK 3u

typedef struct
{
    uint8_t command;
    uint8_t request_length;
    uint8_t reply_length;
    void   *request;
    void   *reply;
    bool    request_seen;
    bool    reply_seen;
} interboard_slot_t;

static interboard_link_t link;
static interboard_slot_t commands[INTERBOARD_CMD_MAX];
static uint8_t command_count;
static uint8_t stream[INTERBOARD_STREAM_SIZE];
static uint32_t stream_count;
static uint32_t last_ok_ms;

#if defined(CONFIG_CDD_INTERBOARD_MASTER)
static uint8_t pending_command;
static uint8_t pending_payload[INTERBOARD_PAYLOAD_MAX];
static uint8_t pending_length;
static bool pending;
static bool in_flight;
static uint8_t in_flight_command;
static uint8_t in_flight_sequence;
static uint32_t in_flight_ms;
static uint8_t next_sequence;
static uint8_t timeout_streak;
#endif

static interboard_slot_t *find_command(uint8_t command)
{
    for (uint8_t index = 0; index < command_count; index++)
    {
        if (commands[index].command == command)
        {
            return &commands[index];
        }
    }
    return NULL;
}

int32_t interboard_bind(const interboard_cmd_t *table, uint8_t count)
{
    if (table == NULL || count == 0u || count > INTERBOARD_CMD_MAX)
    {
        return ERR_INVAL;
    }
    for (uint8_t index = 0; index < count; index++)
    {
        if (table[index].request_length > INTERBOARD_PAYLOAD_MAX || table[index].reply_length > INTERBOARD_PAYLOAD_MAX)
        {
            return ERR_INVAL;
        }
        if ((table[index].request_length != 0u && table[index].request == NULL) ||
            (table[index].reply_length != 0u && table[index].reply == NULL))
        {
            return ERR_INVAL;
        }
        if (find_command(table[index].command) != NULL)
        {
            return ERR_INVAL;
        }
        commands[index].command = table[index].command;
        commands[index].request_length = table[index].request_length;
        commands[index].reply_length = table[index].reply_length;
        commands[index].request = table[index].request;
        commands[index].reply = table[index].reply;
        commands[index].request_seen = false;
        commands[index].reply_seen = false;
        command_count++;
    }
    return ERR_OK;
}

void interboard_get_link(interboard_link_t *out)
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

static bool send_frame(uint8_t command, uint8_t sequence, const void *payload, uint8_t length)
{
    uint8_t frame[INTERBOARD_MAX_FRAME];
    uint32_t frame_length = INTERBOARD_HEADER_LENGTH + (uint32_t)length + INTERBOARD_CRC_LENGTH;

    frame[0] = INTERBOARD_SOF;
    frame[1] = length;
    frame[2] = command;
    frame[3] = sequence;
    if (length != 0u)
    {
        memcpy(frame + INTERBOARD_HEADER_LENGTH, payload, length);
    }
    crc16_append(frame, frame_length);
    return bsp_uart_write(UART_INTERBOARD, frame, frame_length) == (int32_t)frame_length;
}

static void drop_bytes(uint32_t count)
{
    stream_count -= count;
    memmove(stream, stream + count, stream_count);
}

static bool copy_out(const interboard_slot_t *slot, bool seen, uint8_t expect, const void *cache, void *payload, uint8_t length)
{
    if (!seen || length != expect || (length != 0u && (payload == NULL || cache == NULL)))
    {
        return false;
    }
    if (length != 0u)
    {
        memcpy(payload, cache, length);
    }
    (void)slot;
    return true;
}

#if defined(CONFIG_CDD_INTERBOARD_MASTER)
int32_t interboard_post(uint8_t command, const void *payload, uint8_t length)
{
    interboard_slot_t *slot = find_command(command);

    if (slot == NULL || length != slot->request_length || (payload == NULL && length != 0u))
    {
        return ERR_INVAL;
    }

    uint32_t irq_key = os_irq_save();

    pending_command = command;
    pending_length = length;
    if (length != 0u)
    {
        memcpy(pending_payload, payload, length);
        if (slot->request != NULL)
        {
            memcpy(slot->request, payload, length);
        }
    }
    pending = true;
    os_irq_restore(irq_key);
    return ERR_OK;
}

bool interboard_read(uint8_t command, void *payload, uint8_t length)
{
    uint32_t irq_key = os_irq_save();
    interboard_slot_t *slot = find_command(command);
    bool ok = slot != NULL && copy_out(slot, slot->reply_seen, slot->reply_length, slot->reply, payload, length);

    os_irq_restore(irq_key);
    return ok;
}

static void try_send(void)
{
    uint8_t command;
    uint8_t payload[INTERBOARD_PAYLOAD_MAX];
    uint8_t length;
    uint8_t sequence;

    if (in_flight)
    {
        return;
    }

    uint32_t irq_key = os_irq_save();

    if (!pending)
    {
        os_irq_restore(irq_key);
        return;
    }
    command = pending_command;
    length = pending_length;
    if (length != 0u)
    {
        memcpy(payload, pending_payload, length);
    }
    pending = false;
    sequence = next_sequence;
    os_irq_restore(irq_key);

    if (!send_frame(command, sequence, length != 0u ? payload : NULL, length))
    {
        irq_key = os_irq_save();
        if (!pending)
        {
            pending_command = command;
            pending_length = length;
            if (length != 0u)
            {
                memcpy(pending_payload, payload, length);
            }
            pending = true;
        }
        os_irq_restore(irq_key);
        return;
    }
    in_flight = true;
    in_flight_command = command;
    in_flight_sequence = sequence;
    in_flight_ms = os_time_ms();
    next_sequence++;
}

static void master_on_frame(uint8_t command, uint8_t sequence, const uint8_t *payload, uint8_t length)
{
    interboard_slot_t *slot;

    if (!in_flight || command != in_flight_command || sequence != in_flight_sequence)
    {
        return;
    }
    slot = find_command(command);
    if (slot == NULL || length != slot->reply_length)
    {
        note_bad_frame();
        in_flight = false;
        return;
    }
    in_flight = false;
    timeout_streak = 0;
    last_ok_ms = os_time_ms();

    uint32_t irq_key = os_irq_save();

    if (length != 0u && slot->reply != NULL)
    {
        memcpy(slot->reply, payload, length);
    }
    slot->reply_seen = true;
    link.online = true;
    link.frames++;
    os_irq_restore(irq_key);
    try_send();
}

static void master_age(void)
{
    if (!in_flight || (os_time_ms() - in_flight_ms) < INTERBOARD_REPLY_MS)
    {
        return;
    }
    in_flight = false;
    timeout_streak++;

    uint32_t irq_key = os_irq_save();

    link.timeouts++;
    if (timeout_streak >= INTERBOARD_OFFLINE_STREAK)
    {
        link.online = false;
    }
    os_irq_restore(irq_key);
}
#elif defined(CONFIG_CDD_INTERBOARD_SLAVE)
int32_t interboard_write(uint8_t command, const void *payload, uint8_t length)
{
    interboard_slot_t *slot = find_command(command);

    if (slot == NULL || length != slot->reply_length || (payload == NULL && length != 0u))
    {
        return ERR_INVAL;
    }

    uint32_t irq_key = os_irq_save();

    if (length != 0u && slot->reply != NULL)
    {
        memcpy(slot->reply, payload, length);
    }
    slot->reply_seen = true;
    os_irq_restore(irq_key);
    return ERR_OK;
}

bool interboard_read(uint8_t command, void *payload, uint8_t length)
{
    uint32_t irq_key = os_irq_save();
    interboard_slot_t *slot = find_command(command);
    bool ok = slot != NULL && copy_out(slot, slot->request_seen, slot->request_length, slot->request, payload, length);

    os_irq_restore(irq_key);
    return ok;
}

static void slave_on_frame(uint8_t command, uint8_t sequence, const uint8_t *payload, uint8_t length)
{
    uint8_t reply[INTERBOARD_PAYLOAD_MAX];
    uint8_t reply_length;
    interboard_slot_t *slot = find_command(command);

    if (slot == NULL || length != slot->request_length)
    {
        note_bad_frame();
        return;
    }

    uint32_t irq_key = os_irq_save();

    if (length != 0u && slot->request != NULL)
    {
        memcpy(slot->request, payload, length);
    }
    slot->request_seen = true;
    reply_length = slot->reply_length;
    if (reply_length != 0u && slot->reply != NULL)
    {
        memcpy(reply, slot->reply, reply_length);
    }
    os_irq_restore(irq_key);

    if (!send_frame(command, sequence, reply_length != 0u ? reply : NULL, reply_length))
    {
        return;
    }

    irq_key = os_irq_save();
    link.online = true;
    link.frames++;
    os_irq_restore(irq_key);
    last_ok_ms = os_time_ms();
}

static void slave_age(void)
{
    if (last_ok_ms == 0u || (os_time_ms() - last_ok_ms) < INTERBOARD_OFFLINE_GAP_MS)
    {
        return;
    }

    uint32_t irq_key = os_irq_save();

    if (link.online)
    {
        link.timeouts++;
        link.online = false;
    }
    os_irq_restore(irq_key);
}
#endif

static void accept_frame(uint8_t command, uint8_t sequence, const uint8_t *payload, uint8_t length)
{
#if defined(CONFIG_CDD_INTERBOARD_MASTER)
    master_on_frame(command, sequence, payload, length);
#elif defined(CONFIG_CDD_INTERBOARD_SLAVE)
    slave_on_frame(command, sequence, payload, length);
#endif
}

static void parse_stream(void)
{
    while (stream_count > 0u)
    {
        if (stream[0] != INTERBOARD_SOF)
        {
            uint32_t skip = 1u;

            while (skip < stream_count && stream[skip] != INTERBOARD_SOF)
            {
                skip++;
            }
            drop_bytes(skip);
            continue;
        }
        if (stream_count < INTERBOARD_HEADER_LENGTH)
        {
            return;
        }

        uint8_t payload_length = stream[1];

        if (payload_length > INTERBOARD_PAYLOAD_MAX)
        {
            note_bad_frame();
            drop_bytes(1u);
            continue;
        }

        uint32_t frame_length = INTERBOARD_HEADER_LENGTH + (uint32_t)payload_length + INTERBOARD_CRC_LENGTH;

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
        accept_frame(stream[2], stream[3], stream + INTERBOARD_HEADER_LENGTH, payload_length);
        drop_bytes(frame_length);
    }
}

static void interboard_run(void *argument, const uint8_t *data, uint16_t len)
{
    (void)argument;
    if (data == NULL)
    {
#if defined(CONFIG_CDD_INTERBOARD_MASTER)
        master_age();
        try_send();
#elif defined(CONFIG_CDD_INTERBOARD_SLAVE)
        slave_age();
#endif
        return;
    }
    if (len > INTERBOARD_STREAM_SIZE - stream_count)
    {
        stream_count = 0u;
        note_bad_frame();
    }
    memcpy(stream + stream_count, data, len);
    stream_count += len;
    parse_stream();
}
EVENT_TASK_DEFINE(interboard_task, OS_PRIORITY_HIGH, OS_STACK_MEDIUM, EVENT_QUEUE, INTERBOARD_RX_BLOCK_MAX,
                  OS_QUEUE_LEN_4, OS_TIMEOUT_10MS, interboard_run, NULL);

#define INTERBOARD_DEFINE(name, id, request_type, reply_type) \
    static request_type interboard_##name##_request;          \
    static reply_type interboard_##name##_reply;
INTERBOARD_CMD_TABLE(INTERBOARD_DEFINE)
#undef INTERBOARD_DEFINE

#define INTERBOARD_ROW(name, id, request_type, reply_type)                                      \
    {                                                                                           \
        (id),                                                                                   \
        (uint8_t)sizeof(interboard_##name##_request),                                           \
        (uint8_t)sizeof(interboard_##name##_reply),                                             \
        sizeof(interboard_##name##_request) ? (void *)&interboard_##name##_request : NULL,     \
        sizeof(interboard_##name##_reply) ? (void *)&interboard_##name##_reply : NULL,         \
    },

static const interboard_cmd_t interboard_cmds[] =
{
    INTERBOARD_CMD_TABLE(INTERBOARD_ROW)
};
#undef INTERBOARD_ROW

static int32_t interboard_init(void)
{
    int32_t bind_result = interboard_bind(interboard_cmds, (uint8_t)(sizeof interboard_cmds / sizeof interboard_cmds[0]));

    if (bind_result != ERR_OK)
    {
        return bind_result;
    }
    bsp_uart_bind_event(UART_INTERBOARD, &interboard_task);
    return ERR_OK;
}
INIT_CDD_STAGE1(interboard_init);
