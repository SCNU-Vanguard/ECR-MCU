#include <string.h>
#include "cdd/imgtrans/imgtrans.h"
#include "bsp/uart/bsp_uart.h"
#include "osal/event.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/crc/crc.h"

#define IMGTRANS_FRAME_LEN      21u
#define IMGTRANS_HEADER_0       0xA9u
#define IMGTRANS_HEADER_1       0x53u
#define IMGTRANS_CHANNEL_MID    1024
#define IMGTRANS_CHANNEL_LIMIT  660         /* 减中值后的合法范围；超出的轴记 0（和原 VT03 解析一样） */
#define IMGTRANS_TIMEOUT_MS     100u
#define IMGTRANS_RX_BLOCK_MAX   512u        /* 和 board.c 里 UART_IMGTRANS 的接收缓冲一致 */
/* 重组缓冲：等下半帧时最多留 FRAME_LEN - 1 字节，再加一整块新数据 */
#define IMGTRANS_STREAM_SIZE    (IMGTRANS_FRAME_LEN - 1u + IMGTRANS_RX_BLOCK_MAX)

static imgtrans_data_t current_data;        /* 只在 imgtrans_task 里写，别的任务用 imgtrans_get() 取快照 */
static uint8_t  stream[IMGTRANS_STREAM_SIZE] SRAM_NONCACHEABLE;
static uint32_t stream_count;

static int16_t axis(int value)
{
    int centered = value - IMGTRANS_CHANNEL_MID;

    return (centered > IMGTRANS_CHANNEL_LIMIT || centered < -IMGTRANS_CHANNEL_LIMIT) ? 0 : (int16_t)centered;
}

bool imgtrans_parse(const uint8_t *frame, imgtrans_data_t *out) 
{
    if (frame[0] != IMGTRANS_HEADER_0 || frame[1] != IMGTRANS_HEADER_1 || !crc16_verify(frame, IMGTRANS_FRAME_LEN))
    {
        return false;
    }
    out->channel[0] = axis((frame[2] | (frame[3] << 8)) & 0x07FF);                                          /* 右横 */
    out->channel[1] = axis(((frame[3] >> 3) | (frame[4] << 5)) & 0x07FF);                                   /* 右竖 */
    out->channel[2] = axis(((frame[4] >> 6) | (frame[5] << 2) | (frame[6] << 10)) & 0x07FF);               /* 左横 */
    out->channel[3] = axis(((frame[6] >> 1) | (frame[7] << 7)) & 0x07FF);                                   /* 左竖 */
    out->wheel = axis(((frame[8] >> 1) | (frame[9] << 7)) & 0x07FF);
    out->gear = (uint8_t)((frame[7] >> 4) & 0x03u);
    out->pause = (uint8_t)((frame[7] >> 6) & 0x01u);
    out->custom_left = (uint8_t)((frame[7] >> 7) & 0x01u);
    out->custom_right = (uint8_t)(frame[8] & 0x01u);
    out->trigger = (uint8_t)((frame[9] >> 4) & 0x01u);
    out->mouse_x = (int16_t)(frame[10] | (frame[11] << 8));
    out->mouse_y = (int16_t)(frame[12] | (frame[13] << 8));
    out->mouse_z = (int16_t)(frame[14] | (frame[15] << 8));
    out->mouse_left = (uint8_t)(frame[16] & 0x03u);
    out->mouse_right = (uint8_t)((frame[16] >> 2) & 0x03u);
    out->mouse_middle = (uint8_t)((frame[16] >> 4) & 0x03u);
    out->key = (uint16_t)(frame[17] | (frame[18] << 8));
    return true;
}

static void drop_bytes(uint32_t count)
{
    stream_count -= count;
    memmove(stream, stream + count, stream_count);
}

/* 在 stream 里找帧：0xA9 0x53 开头、凑满 21 字节、CRC16 通过。
 * CRC 失败只丢 1 字节再找（假帧头可能压在真帧里面） */
static void parse_stream(void)
{
    while (stream_count > 0u)
    {
        if (stream[0] != IMGTRANS_HEADER_0)
        {
            uint32_t skip = 1u;

            while (skip < stream_count && stream[skip] != IMGTRANS_HEADER_0)
            {
                skip++;
            }
            drop_bytes(skip);
            continue;
        }
        if (stream_count < 2u)
        {
            return;
        }
        if (stream[1] != IMGTRANS_HEADER_1)
        {
            drop_bytes(1u);
            continue;
        }
        if (stream_count < IMGTRANS_FRAME_LEN)
        {
            return;                                 /* 等下一块数据 */
        }

        imgtrans_data_t new_data = current_data;    /* current_data 只在本任务里写，这里读不用加锁 */

        if (imgtrans_parse(stream, &new_data))
        {
            new_data.online = true;
            new_data.frames++;

            uint32_t irq_key = os_irq_save();

            current_data = new_data;
            os_irq_restore(irq_key);
            drop_bytes(IMGTRANS_FRAME_LEN);
        }
        else
        {
            current_data.bad_frames++;
            drop_bytes(1u);
        }
    }
}

/* 事件任务：UART 每次收到的数据块（≤ 512 字节，DMA 循环模式，块边界和帧边界不对齐）拼进字节流缓冲再解析。
 *   EVENT_QUEUE：每个块都要处理，不能丢，否则字节流断了
 *   超时 100 ms：data == NULL，视为离线——控制量清零，计数保留 */
static void imgtrans_run(void *argument, const uint8_t *data, uint16_t len)
{
    (void)argument;
    if (data == NULL)
    {
        uint32_t irq_key = os_irq_save();
        uint32_t frames = current_data.frames;
        uint32_t bad_frames = current_data.bad_frames;

        memset(&current_data, 0, sizeof current_data);
        current_data.frames = frames;
        current_data.bad_frames = bad_frames;
        os_irq_restore(irq_key);
        stream_count = 0u;
        return;
    }
    if (len > IMGTRANS_STREAM_SIZE - stream_count)
    {
        stream_count = 0u;
        current_data.bad_frames++;
    }
    memcpy(stream + stream_count, data, len);
    stream_count += len;
    parse_stream();
}
EVENT_TASK_DEFINE(imgtrans_task, OS_PRIORITY_HIGH, OS_STACK_MEDIUM, EVENT_QUEUE, IMGTRANS_RX_BLOCK_MAX, 2,
                  IMGTRANS_TIMEOUT_MS, imgtrans_run, NULL);

void imgtrans_get(imgtrans_data_t *out)
{
    uint32_t irq_key = os_irq_save();

    *out = current_data;
    os_irq_restore(irq_key);
}

static int32_t imgtrans_init(void)
{
    memset(&current_data, 0, sizeof current_data);
    bsp_uart_bind_event(UART_IMGTRANS, &imgtrans_task);
    return ERR_OK;
}
INIT_CDD_STAGE1(imgtrans_init);

#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"

/* imgtrans：打印摇杆、拨轮、挡位、键鼠和收包统计 */
static int shell_imgtrans(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    static imgtrans_data_t data;

    imgtrans_get(&data);
    shell_out("imgtrans: online=%d frames=%lu bad=%lu\r\n", (int)data.online, (unsigned long)data.frames,
              (unsigned long)data.bad_frames);
    shell_out("  ch=%d %d %d %d wheel=%d gear=%u pause=%u\r\n", (int)data.channel[0], (int)data.channel[1],
              (int)data.channel[2], (int)data.channel[3], (int)data.wheel, (unsigned)data.gear, (unsigned)data.pause);
    shell_out("  custom_l=%u custom_r=%u trigger=%u key=0x%04x\r\n", (unsigned)data.custom_left,
              (unsigned)data.custom_right, (unsigned)data.trigger, (unsigned)data.key);
    shell_out("  mouse=%d %d %d btn=%u %u %u\r\n", (int)data.mouse_x, (int)data.mouse_y, (int)data.mouse_z,
              (unsigned)data.mouse_left, (unsigned)data.mouse_right, (unsigned)data.mouse_middle);
    shell_out("  event: posted=%lu handled=%lu dropped=%lu\r\n", (unsigned long)imgtrans_task.posted,
              (unsigned long)imgtrans_task.handled, (unsigned long)imgtrans_task.dropped);
    return 0;
}
SHELL_CMD(imgtrans, shell_imgtrans, show_vt03_link_remote)
#endif
