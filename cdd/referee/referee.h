#ifndef CDD_REFEREE_H__
#define CDD_REFEREE_H__
#include "component/common/types.h"
#include "cdd/referee/referee_protocol.h"

/* 裁判系统（UART_REFEREE）：接收解析 + 发送。
 *   接收：referee_task 把串口数据当字节流（DMA 循环模式下一帧可能被切成两段），按帧头 CRC8 / 整帧 CRC16 找帧，
 *         合法的帧按命令码存进 referee_info_t。其他任务用 referee_get 取一份一致的快照。
 *   发送：referee_send_interaction 发 0x0301 机器人交互（UI 绘图、机器人间通信）。UI 绘图原语在 cdd/referee/ui。
 * 裁判系统离线（1 s 没收到合法帧）时 online = false，除计数外所有数据清零。 */

typedef struct
{
    uint8_t  robot_color;           /* referee_color_e */
    uint16_t robot_id;              /* 本机器人 ID；0 = 还没收到 0x0201，UI 和机器人间通信都不能发 */
    uint16_t client_id;             /* 本机器人对应的选手端 ID */
} referee_id_t;

typedef struct
{
    referee_id_t                    id;
    referee_game_state_t            game_state;             /* 0x0001 */
    referee_game_result_t           game_result;            /* 0x0002 */
    referee_game_robot_hp_t         robot_hp;               /* 0x0003 */
    referee_event_data_t            event;                  /* 0x0101 */
    referee_warning_t               warning;                /* 0x0104 */
    referee_dart_info_t             dart;                   /* 0x0105 */
    referee_robot_performance_t     performance;            /* 0x0201 */
    referee_power_heat_t            power_heat;             /* 0x0202 */
    referee_robot_position_t        position;               /* 0x0203 */
    referee_buff_t                  buff;                   /* 0x0204 */
    referee_robot_hurt_t            hurt;                   /* 0x0206 */
    referee_shoot_data_t            shoot;                  /* 0x0207 */
    referee_projectile_allowance_t  projectile_allowance;   /* 0x0208 */
    referee_rfid_status_t           rfid;                   /* 0x0209 */
    referee_dart_launch_status_t    dart_launch;            /* 0x020A */
    referee_ground_robot_position_t ground_robot_position;  /* 0x020B */
    referee_radar_mark_t            radar_mark;             /* 0x020C */
    referee_sentry_info_t           sentry;                 /* 0x020D */
    referee_radar_info_t            radar;                  /* 0x020E */
    referee_robot_interaction_t     interaction;            /* 0x0301 */
    referee_map_command_t           map_command;            /* 0x0303 */
    bool                            online;
    uint32_t                        frames;                 /* 合法帧数 */
    uint32_t                        bad_frames;             /* 帧头 CRC8 或整帧 CRC16 不对的次数 */
} referee_info_t;

void referee_get(referee_info_t *out);              /* 取一份一致的快照（任务里调用） */
bool referee_get_id(referee_id_t *out);             /* 本机器人 ID；还没收到 0x0201 时返回 false */

/* 发一帧：自动填帧头、包序号、CRC8、CRC16。内容长度不超过 REFEREE_SEND_DATA_MAX。
 * 非阻塞：返回 ERR_OK 已入发送队列；ERR_BUSY 发送队列放不下整帧（不会只发半帧）；ERR_INVAL 参数不对 */
#define REFEREE_SEND_FRAME_MAX      128u
#define REFEREE_SEND_DATA_MAX       (REFEREE_SEND_FRAME_MAX - REFEREE_FRAME_OVERHEAD)
int32_t referee_send_frame(uint16_t command_id, const void *data, uint16_t length);

/* 发一帧 0x0301：交互头（sender 取本机器人 ID）+ 内容。content_length 不超过 REFEREE_SEND_DATA_MAX - 6。
 * data_command_id 是 referee_interaction_id_e；UI 绘图的 receiver_id 填 client_id。还没收到本机器人 ID 时返回 ERR_BUSY。
 * 0x0301 整体上行频率上限 10 Hz，这里不限速，由调用者安排 */
int32_t referee_send_interaction(uint16_t data_command_id, uint16_t receiver_id, const void *content, uint16_t content_length);
#endif
