#include <stddef.h>
#include <string.h>
#include "cdd/referee/referee.h"
#include "bsp/uart/bsp_uart.h"
#include "osal/event.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/common/cqueue.h"
#include "component/crc/crc.h"

#define REFEREE_TIMEOUT_MS          1000u       /* 超过这么久没收到合法帧 = 离线 */
#define REFEREE_MAX_DATA_LENGTH     300u        /* 协议里最长的数据段不到这个数；帧头长度超过它一定是误同步 */
#define REFEREE_MAX_FRAME_LENGTH    (REFEREE_MAX_DATA_LENGTH + REFEREE_FRAME_OVERHEAD)
#define REFEREE_RX_BLOCK_MAX        256u        /* 和 board.c 里 UART_REFEREE 的接收缓冲一致 */
/* 重组缓冲：等下半帧时最多留 MAX_FRAME_LENGTH - 1 字节，再加一整块新数据 */
#define REFEREE_STREAM_SIZE         (REFEREE_MAX_FRAME_LENGTH + REFEREE_RX_BLOCK_MAX)

static referee_info_t current_info;             /* 只在 referee_task 里写，别的任务用 referee_get() 取快照 */
static uint8_t  stream[REFEREE_STREAM_SIZE];    /* 字节流重组缓冲，只在 referee_task 里用 */
static uint32_t stream_count;
static uint8_t  tx_sequence;                    /* 发送包序号 */

/* ---------------- 命令码 -> 存放位置 ----------------
 * 数据段至少要有 minimum_length 字节才接收；存放时先把成员清零，再拷 min(data_length, sizeof 成员) 字节，
 * 所以协议后续版本加长数据段不会越界，缩短的部分保持 0。
 * 协议长度和结构大小不一致的命令（0x0203、0x0208、0x0301）minimum_length 取两者较小者 */
#define MIN_OF(_a, _b)  ((_a) < (_b) ? (_a) : (_b))
#define REFEREE_ENTRY(_command, _length, _member)                                           \
    {                                                                                       \
        (_command),                                                                         \
        MIN_OF((_length), sizeof(((referee_info_t *)0)->_member)),                          \
        offsetof(referee_info_t, _member),                                                  \
        sizeof(((referee_info_t *)0)->_member),                                             \
    }

typedef struct
{
    uint16_t command_id;
    uint16_t minimum_length;
    uint16_t member_offset;
    uint16_t member_size;
} referee_entry_t;

static const referee_entry_t referee_table[] =
{
    REFEREE_ENTRY(REFEREE_CMD_GAME_STATE,            REFEREE_LENGTH_GAME_STATE,            game_state),
    REFEREE_ENTRY(REFEREE_CMD_GAME_RESULT,           REFEREE_LENGTH_GAME_RESULT,           game_result),
    REFEREE_ENTRY(REFEREE_CMD_GAME_ROBOT_HP,         REFEREE_LENGTH_GAME_ROBOT_HP,         robot_hp),
    REFEREE_ENTRY(REFEREE_CMD_EVENT_DATA,            REFEREE_LENGTH_EVENT_DATA,            event),
    REFEREE_ENTRY(REFEREE_CMD_REFEREE_WARNING,       REFEREE_LENGTH_REFEREE_WARNING,       warning),
    REFEREE_ENTRY(REFEREE_CMD_DART_INFO,             REFEREE_LENGTH_DART_INFO,             dart),
    REFEREE_ENTRY(REFEREE_CMD_ROBOT_PERFORMANCE,     REFEREE_LENGTH_ROBOT_PERFORMANCE,     performance),
    REFEREE_ENTRY(REFEREE_CMD_POWER_HEAT,            REFEREE_LENGTH_POWER_HEAT,            power_heat),
    REFEREE_ENTRY(REFEREE_CMD_ROBOT_POSITION,        REFEREE_LENGTH_ROBOT_POSITION,        position),
    REFEREE_ENTRY(REFEREE_CMD_BUFF,                  REFEREE_LENGTH_BUFF,                  buff),
    REFEREE_ENTRY(REFEREE_CMD_ROBOT_HURT,            REFEREE_LENGTH_ROBOT_HURT,            hurt),
    REFEREE_ENTRY(REFEREE_CMD_SHOOT_DATA,            REFEREE_LENGTH_SHOOT_DATA,            shoot),
    REFEREE_ENTRY(REFEREE_CMD_PROJECTILE_ALLOWANCE,  REFEREE_LENGTH_PROJECTILE_ALLOWANCE,  projectile_allowance),
    REFEREE_ENTRY(REFEREE_CMD_RFID_STATUS,           REFEREE_LENGTH_RFID_STATUS,           rfid),
    REFEREE_ENTRY(REFEREE_CMD_DART_LAUNCH_STATUS,    REFEREE_LENGTH_DART_LAUNCH_STATUS,    dart_launch),
    REFEREE_ENTRY(REFEREE_CMD_GROUND_ROBOT_POSITION, REFEREE_LENGTH_GROUND_ROBOT_POSITION, ground_robot_position),
    REFEREE_ENTRY(REFEREE_CMD_RADAR_MARK,            REFEREE_LENGTH_RADAR_MARK,            radar_mark),
    REFEREE_ENTRY(REFEREE_CMD_SENTRY_INFO,           REFEREE_LENGTH_SENTRY_INFO,           sentry),
    REFEREE_ENTRY(REFEREE_CMD_RADAR_INFO,            REFEREE_LENGTH_RADAR_INFO,            radar),
    REFEREE_ENTRY(REFEREE_CMD_ROBOT_INTERACTION,     REFEREE_INTERACTION_HEADER_LENGTH,    interaction),
    REFEREE_ENTRY(REFEREE_CMD_MAP_COMMAND,           REFEREE_LENGTH_MAP_COMMAND,           map_command),
};

static void update_id(void)
{
    uint8_t robot_id = current_info.performance.robot_id;

    current_info.id.robot_id = robot_id;
    current_info.id.robot_color = (robot_id > 100u) ? REFEREE_COLOR_BLUE : REFEREE_COLOR_RED;
    current_info.id.client_id = (uint16_t)(REFEREE_CLIENT_ID_BASE + robot_id);
}

/* 处理一帧合法帧（CRC 已校验）。整个更新放在临界区里，别的任务取快照不会拿到写了一半的数据 */
static void dispatch(uint16_t command_id, const uint8_t *data, uint16_t data_length)
{
    uint32_t irq_key = os_irq_save();

    current_info.online = true;
    current_info.frames++;
    for (uint32_t i = 0; i < sizeof referee_table / sizeof referee_table[0]; i++)
    {
        const referee_entry_t *entry = &referee_table[i];

        if (entry->command_id != command_id)
        {
            continue;
        }
        if (data_length >= entry->minimum_length)
        {
            uint8_t *member = (uint8_t *)&current_info + entry->member_offset;

            memset(member, 0, entry->member_size);
            memcpy(member, data, MIN_OF(data_length, entry->member_size));
            if (command_id == REFEREE_CMD_ROBOT_PERFORMANCE)
            {
                update_id();
            }
        }
        break;                              /* 数据段比协议规定的短：帧本身合法，内容不收 */
    }
    os_irq_restore(irq_key);
}

static void drop_bytes(uint32_t count)
{
    stream_count -= count;
    memmove(stream, stream + count, stream_count);
}

/* 在 stream 里找帧：帧头 CRC8 -> 长度上限 -> 够不够一整帧 -> 整帧 CRC16。
 * 任何一步校验失败都只丢 1 字节再往后找（假 0xA5 可能压在真帧里面） */
static void parse_stream(void)
{
    while (stream_count > 0u)
    {
        if (stream[0] != REFEREE_FRAME_SOF)
        {
            uint32_t skip = 1u;

            while (skip < stream_count && stream[skip] != REFEREE_FRAME_SOF)
            {
                skip++;
            }
            drop_bytes(skip);
            continue;
        }
        if (stream_count < REFEREE_HEADER_LENGTH)
        {
            return;                                 /* 帧头还没收全 */
        }
        if (!crc8_verify(stream, REFEREE_HEADER_LENGTH))
        {
            current_info.bad_frames++;
            drop_bytes(1u);
            continue;
        }

        uint16_t data_length = (uint16_t)(stream[1] | (stream[2] << 8));

        if (data_length > REFEREE_MAX_DATA_LENGTH)
        {
            current_info.bad_frames++;
            drop_bytes(1u);
            continue;
        }

        uint32_t frame_length = (uint32_t)data_length + REFEREE_FRAME_OVERHEAD;

        if (stream_count < frame_length)
        {
            return;                                 /* 等下一块数据 */
        }
        if (!crc16_verify(stream, frame_length))
        {
            current_info.bad_frames++;
            drop_bytes(1u);
            continue;
        }

        uint16_t command_id = (uint16_t)(stream[5] | (stream[6] << 8));

        dispatch(command_id, stream + REFEREE_DATA_OFFSET, data_length);
        drop_bytes(frame_length);
    }
}

/* 事件任务：UART 每次收到的数据块（≤ 256 字节，DMA 循环模式，块边界和帧边界不对齐）拼进字节流缓冲再解析。
 *   EVENT_QUEUE：每个块都要处理，不能丢，否则字节流断了
 *   超时 1 s：data == NULL，视为离线——数据和本机器人 ID 清零，计数保留；再收到合法帧自动恢复 */
static void referee_run(void *argument, const uint8_t *data, uint16_t len)
{
    (void)argument;
    if (data == NULL)
    {
        uint32_t irq_key = os_irq_save();
        uint32_t frames = current_info.frames;
        uint32_t bad_frames = current_info.bad_frames;

        memset(&current_info, 0, sizeof current_info);
        current_info.frames = frames;
        current_info.bad_frames = bad_frames;
        os_irq_restore(irq_key);
        stream_count = 0u;
        return;
    }
    if (len > REFEREE_STREAM_SIZE - stream_count)   /* 正常不会发生（事件任务限制了块大小）；万一发生就丢掉旧数据重新同步 */
    {
        stream_count = 0u;
        current_info.bad_frames++;
    }
    memcpy(stream + stream_count, data, len);
    stream_count += len;
    parse_stream();
}
EVENT_TASK_DEFINE(referee_task, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, EVENT_QUEUE, REFEREE_RX_BLOCK_MAX, 4,
                  REFEREE_TIMEOUT_MS, referee_run, NULL);

void referee_get(referee_info_t *out)
{
    uint32_t irq_key = os_irq_save();

    *out = current_info;
    os_irq_restore(irq_key);
}

bool referee_get_id(referee_id_t *out)
{
    uint32_t irq_key = os_irq_save();

    *out = current_info.id;
    os_irq_restore(irq_key);
    return out->robot_id != 0u;
}

/* ---------------- 发送 ---------------- */
int32_t referee_send_frame(uint16_t command_id, const void *data, uint16_t length)
{
    uint8_t frame[REFEREE_SEND_FRAME_MAX];

    if ((data == NULL && length != 0u) || length > REFEREE_SEND_DATA_MAX)
    {
        return ERR_INVAL;
    }

    uint32_t frame_length = (uint32_t)length + REFEREE_FRAME_OVERHEAD;

    /* 查剩余空间、取包序号、入队放在同一段关中断里。
     * bsp_uart_write 队列不够时会只入队一部分，半帧对裁判系统只是垃圾；关中断期间别的任务插不进来 */
    uint32_t irq_key = os_irq_save();

    if (cqueue_unused(&bsp_uart_state(UART_REFEREE)->tx_queue) < frame_length)
    {
        os_irq_restore(irq_key);
        return ERR_BUSY;
    }
    frame[0] = REFEREE_FRAME_SOF;
    frame[1] = (uint8_t)(length & 0xFFu);
    frame[2] = (uint8_t)(length >> 8);
    frame[3] = tx_sequence++;
    crc8_append(frame, REFEREE_HEADER_LENGTH);
    frame[5] = (uint8_t)(command_id & 0xFFu);
    frame[6] = (uint8_t)(command_id >> 8);
    if (length != 0u)
    {
        memcpy(frame + REFEREE_DATA_OFFSET, data, length);
    }
    crc16_append(frame, frame_length);

    int32_t queued = bsp_uart_write(UART_REFEREE, frame, frame_length);

    os_irq_restore(irq_key);
    return (queued == (int32_t)frame_length) ? ERR_OK : ERR_BUSY;
}

int32_t referee_send_interaction(uint16_t data_command_id, uint16_t receiver_id, const void *content, uint16_t content_length)
{
    uint8_t payload[REFEREE_SEND_DATA_MAX];
    referee_id_t id;

    if ((content == NULL && content_length != 0u) || content_length > REFEREE_SEND_DATA_MAX - REFEREE_INTERACTION_HEADER_LENGTH)
    {
        return ERR_INVAL;
    }
    if (!referee_get_id(&id))
    {
        return ERR_BUSY;
    }

    referee_interaction_header_t header = { data_command_id, id.robot_id, receiver_id };

    memcpy(payload, &header, sizeof header);
    if (content_length != 0u)
    {
        memcpy(payload + sizeof header, content, content_length);
    }
    return referee_send_frame(REFEREE_CMD_ROBOT_INTERACTION, payload, (uint16_t)(sizeof header + content_length));
}

static int32_t referee_init(void)
{
    memset(&current_info, 0, sizeof current_info);
    bsp_uart_bind_event(UART_REFEREE, &referee_task);
    return ERR_OK;
}
INIT_CDD_STAGE1(referee_init);

#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"

/* referee：打印在线状态、本机器人 ID、比赛阶段、血量、热量、功率限制等关键量和收发统计 */
static int shell_referee(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    static referee_info_t info;     /* shell 任务里用，放静态区不占栈；shell 命令串行执行 */

    referee_get(&info);
    shell_out("referee: online=%d frames=%lu bad=%lu\r\n", (int)info.online, (unsigned long)info.frames,
              (unsigned long)info.bad_frames);
    shell_out("  robot_id=%u color=%s client_id=%u level=%u\r\n", (unsigned)info.id.robot_id,
              (info.id.robot_color == REFEREE_COLOR_BLUE) ? "blue" : "red", (unsigned)info.id.client_id,
              (unsigned)info.performance.robot_level);
    shell_out("  game: type=%u progress=%u remain=%us\r\n", (unsigned)info.game_state.game_type,
              (unsigned)info.game_state.game_progress, (unsigned)info.game_state.stage_remain_time);
    shell_out("  hp=%u/%u heat=%u/%u power_limit=%uW buffer=%uJ\r\n", (unsigned)info.performance.current_hp,
              (unsigned)info.performance.maximum_hp, (unsigned)info.power_heat.shooter_17mm_barrel_heat,
              (unsigned)info.performance.shooter_barrel_heat_limit, (unsigned)info.performance.chassis_power_limit,
              (unsigned)info.power_heat.buffer_energy);
    shell_out("  allowance_17mm=%u shoot_speed_x1000=%ld\r\n", (unsigned)info.projectile_allowance.projectile_allowance_17mm,
              (long)(info.shoot.bullet_speed * 1000.0f));
    shell_out("  event: posted=%lu handled=%lu dropped=%lu\r\n", (unsigned long)referee_task.posted,
              (unsigned long)referee_task.handled, (unsigned long)referee_task.dropped);
    return 0;
}
SHELL_CMD(referee, shell_referee, show_referee_status)
#endif
