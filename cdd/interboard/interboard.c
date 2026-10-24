#include <string.h>
#include "cdd/interboard/interboard.h"
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

#define INTERBOARD_SOF            0xA5u
#define INTERBOARD_RX_BLOCK_MAX   128u    /* 与 board.c 里 interboard 接收缓冲一致 */
#define INTERBOARD_HEADER_LENGTH  4u      /* 帧头 + 长度 + 命令 + 序号 */
#define INTERBOARD_CRC_LENGTH     2u
#define INTERBOARD_MAX_FRAME      (INTERBOARD_HEADER_LENGTH + INTERBOARD_PAYLOAD_MAX + INTERBOARD_CRC_LENGTH)
#define INTERBOARD_STREAM_SIZE    (INTERBOARD_MAX_FRAME - 1u + INTERBOARD_RX_BLOCK_MAX)
#define INTERBOARD_REPLY_MS       10u
#define INTERBOARD_OFFLINE_GAP_MS 50u
#define INTERBOARD_OFFLINE_STREAK 3u

static interboard_link_t link;
static interboard_request_t request_image;
static interboard_reply_t reply_image;
#if defined(CONFIG_CDD_INTERBOARD_MASTER)
static bool reply_seen;
#else
static bool request_seen;
#endif
static uint8_t stream[INTERBOARD_STREAM_SIZE];
static uint32_t stream_count;
static uint32_t last_ok_ms;

#if defined(CONFIG_CDD_INTERBOARD_MASTER)
static bool pending;
static bool in_flight;
static uint8_t in_flight_sequence;
static uint32_t in_flight_ms;
static uint8_t next_sequence;
static uint8_t timeout_streak;
#endif

#if defined(CONFIG_CDD_INTERBOARD_MASTER)
void interboard_load_request(interboard_request_t *request)
{
    if (request == NULL)
    {
        return;
    }

    uint32_t irq_key = os_irq_save();

    *request = request_image;
    os_irq_restore(irq_key);
}

void interboard_store_request(const interboard_request_t *request)
{
    if (request == NULL)
    {
        return;
    }

    uint32_t irq_key = os_irq_save();

    request_image = *request;
    os_irq_restore(irq_key);
}

bool interboard_load_reply(interboard_reply_t *reply)
{
    bool seen;

    if (reply == NULL)
    {
        return false;
    }

    uint32_t irq_key = os_irq_save();

    seen = reply_seen;
    if (seen)
    {
        *reply = reply_image;
    }
    os_irq_restore(irq_key);
    return seen;
}
#elif defined(CONFIG_CDD_INTERBOARD_SLAVE)
bool interboard_load_request(interboard_request_t *request)
{
    bool seen;

    if (request == NULL)
    {
        return false;
    }

    uint32_t irq_key = os_irq_save();

    seen = request_seen;
    if (seen)
    {
        *request = request_image;
    }
    os_irq_restore(irq_key);
    return seen;
}

void interboard_load_reply(interboard_reply_t *reply)
{
    if (reply == NULL)
    {
        return;
    }

    uint32_t irq_key = os_irq_save();

    *reply = reply_image;
    os_irq_restore(irq_key);
}

void interboard_store_reply(const interboard_reply_t *reply)
{
    if (reply == NULL)
    {
        return;
    }

    uint32_t irq_key = os_irq_save();

    reply_image = *reply;
    os_irq_restore(irq_key);
}
#endif

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

#if defined(CONFIG_CDD_INTERBOARD_MASTER)
static void try_send(void);

int32_t interboard_post(void)
{
    if (sizeof(interboard_request_t) > INTERBOARD_PAYLOAD_MAX)
    {
        return ERR_INVAL;
    }

    uint32_t irq_key = os_irq_save();

    pending = true;
    os_irq_restore(irq_key);
    try_send();
    return ERR_OK;
}

static void try_send(void)
{
    interboard_request_t request;
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
    request = request_image;
    pending = false;
    sequence = next_sequence;
    os_irq_restore(irq_key);

    if (!send_frame(INTERBOARD_CMD, sequence, &request, (uint8_t)sizeof request))
    {
        irq_key = os_irq_save();
        pending = true;
        os_irq_restore(irq_key);
        return;
    }
    in_flight = true;
    in_flight_sequence = sequence;
    in_flight_ms = os_time_ms();
    next_sequence++;
}

static void master_on_frame(uint8_t command, uint8_t sequence, const uint8_t *payload, uint8_t length)
{
    if (!in_flight || command != INTERBOARD_CMD || sequence != in_flight_sequence)
    {
        return;
    }
    if (length != sizeof(interboard_reply_t))
    {
        note_bad_frame();
        in_flight = false;
        return;
    }
    in_flight = false;
    timeout_streak = 0;
    last_ok_ms = os_time_ms();

    uint32_t irq_key = os_irq_save();

    memcpy(&reply_image, payload, sizeof reply_image);
    reply_seen = true;
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
static void slave_on_frame(uint8_t command, uint8_t sequence, const uint8_t *payload, uint8_t length)
{
    interboard_reply_t reply;

    if (command != INTERBOARD_CMD || length != sizeof(interboard_request_t))
    {
        note_bad_frame();
        return;
    }

    uint32_t irq_key = os_irq_save();

    memcpy(&request_image, payload, sizeof request_image);
    request_seen = true;
    reply = reply_image;
    os_irq_restore(irq_key);

    if (!send_frame(INTERBOARD_CMD, sequence, &reply, (uint8_t)sizeof reply))
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

static int32_t interboard_init(void)
{
    bsp_uart_bind_event(UART_INTERBOARD, &interboard_task);
    return ERR_OK;
}
INIT_CDD_STAGE1(interboard_init);
