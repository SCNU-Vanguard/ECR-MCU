#ifndef CDD_REFEREE_PROTOCOL_H__
#define CDD_REFEREE_PROTOCOL_H__
/* RoboMaster 裁判系统串口协议（2026）：帧格式、命令码、各命令的数据段结构。
 * 只有协议里的"线上格式"，没有任何逻辑。移植自 2026Standard 的 NEW_referee/referee_protocol.h，
 * 命令码和数据段与 2026_VANGUARD_Omni_Infantry 一致，另外带上 Standard 才有的 0x0303 地图指令和 0x0120 哨兵交互子命令。
 * 所有结构按 1 字节对齐、小端，位域从低位开始排。
 *
 * 帧格式：
 *   帧头 5 字节：0xA5 | 数据段长度（2 字节）| 包序号 | CRC8（前 4 字节）
 *   命令码 2 字节
 *   数据段 data_length 字节
 *   CRC16 2 字节（前面所有字节，包括帧头） */
#include "component/common/types.h"

#define REFEREE_FRAME_SOF               0xA5u
#define REFEREE_HEADER_LENGTH           5u      /* 帧头 */
#define REFEREE_HEADER_CRC8_COVERED     4u      /* 帧头里被 CRC8 校验的前几个字节 */
#define REFEREE_COMMAND_ID_LENGTH       2u
#define REFEREE_TAIL_LENGTH             2u      /* 帧尾 CRC16 */
#define REFEREE_FRAME_OVERHEAD          (REFEREE_HEADER_LENGTH + REFEREE_COMMAND_ID_LENGTH + REFEREE_TAIL_LENGTH)
#define REFEREE_DATA_OFFSET             (REFEREE_HEADER_LENGTH + REFEREE_COMMAND_ID_LENGTH)

/* 收到的 0x0301 机器人交互里，内容段最多保存多少字节（自定义协议可以改这个数；哨兵交互 0x0120 只用 4 字节） */
#define REFEREE_INTERACTION_DATA_SIZE   5u

#pragma pack(push, 1)

/* ---------------- 命令码 ---------------- */
typedef enum
{
    REFEREE_CMD_GAME_STATE              = 0x0001,   /* 比赛状态 */
    REFEREE_CMD_GAME_RESULT             = 0x0002,   /* 比赛结果 */
    REFEREE_CMD_GAME_ROBOT_HP           = 0x0003,   /* 己方机器人血量 */
    REFEREE_CMD_EVENT_DATA              = 0x0101,   /* 场地事件 */
    REFEREE_CMD_REFEREE_WARNING         = 0x0104,   /* 裁判警告 */
    REFEREE_CMD_DART_INFO               = 0x0105,   /* 飞镖发射相关 */
    REFEREE_CMD_ROBOT_PERFORMANCE       = 0x0201,   /* 机器人性能体系（含机器人 ID） */
    REFEREE_CMD_POWER_HEAT              = 0x0202,   /* 实时功率热量 */
    REFEREE_CMD_ROBOT_POSITION          = 0x0203,   /* 机器人位置 */
    REFEREE_CMD_BUFF                    = 0x0204,   /* 机器人增益和底盘能量 */
    REFEREE_CMD_ROBOT_HURT              = 0x0206,   /* 伤害状态 */
    REFEREE_CMD_SHOOT_DATA              = 0x0207,   /* 实时射击 */
    REFEREE_CMD_PROJECTILE_ALLOWANCE    = 0x0208,   /* 允许发弹量 */
    REFEREE_CMD_RFID_STATUS             = 0x0209,   /* RFID 状态 */
    REFEREE_CMD_DART_LAUNCH_STATUS      = 0x020A,   /* 飞镖发射站状态 */
    REFEREE_CMD_GROUND_ROBOT_POSITION   = 0x020B,   /* 地面机器人位置 */
    REFEREE_CMD_RADAR_MARK              = 0x020C,   /* 雷达标记进度 */
    REFEREE_CMD_SENTRY_INFO             = 0x020D,   /* 哨兵信息 */
    REFEREE_CMD_RADAR_INFO              = 0x020E,   /* 雷达信息 */
    REFEREE_CMD_ROBOT_INTERACTION       = 0x0301,   /* 机器人交互（UI 绘图、机器人间通信都走这条） */
    REFEREE_CMD_CUSTOM_CONTROLLER       = 0x0302,   /* 自定义控制器 -> 机器人（只定义，不解析） */
    REFEREE_CMD_MAP_COMMAND             = 0x0303,   /* 选手端小地图下发 */
    REFEREE_CMD_CUSTOM_CLIENT_FROM_ROBOT = 0x0310,  /* 机器人 -> 自定义客户端（只定义，不解析） */
    REFEREE_CMD_CUSTOM_CLIENT_TO_ROBOT  = 0x0311,   /* 自定义客户端 -> 机器人（只定义，不解析） */
} referee_command_e;

/* ---------------- 各命令的数据段长度（协议规定） ---------------- */
#define REFEREE_LENGTH_GAME_STATE           11u
#define REFEREE_LENGTH_GAME_RESULT          1u
#define REFEREE_LENGTH_GAME_ROBOT_HP        16u
#define REFEREE_LENGTH_EVENT_DATA           4u
#define REFEREE_LENGTH_REFEREE_WARNING      3u
#define REFEREE_LENGTH_DART_INFO            3u
#define REFEREE_LENGTH_ROBOT_PERFORMANCE    13u
#define REFEREE_LENGTH_POWER_HEAT           14u
#define REFEREE_LENGTH_ROBOT_POSITION       16u     /* 协议长度 16，已知字段只有前 12 字节（x、y、angle） */
#define REFEREE_LENGTH_BUFF                 8u
#define REFEREE_LENGTH_ROBOT_HURT           1u
#define REFEREE_LENGTH_SHOOT_DATA           7u
#define REFEREE_LENGTH_PROJECTILE_ALLOWANCE 6u      /* 协议长度 6；结构里多留了 fortress 一项，收到 6 字节时它保持 0 */
#define REFEREE_LENGTH_RFID_STATUS          5u
#define REFEREE_LENGTH_DART_LAUNCH_STATUS   6u
#define REFEREE_LENGTH_GROUND_ROBOT_POSITION 40u
#define REFEREE_LENGTH_RADAR_MARK           2u
#define REFEREE_LENGTH_SENTRY_INFO          6u
#define REFEREE_LENGTH_RADAR_INFO           1u
#define REFEREE_LENGTH_MAP_COMMAND          12u

/* ---------------- 数据段结构 ---------------- */
/* 0x0001 比赛状态 */
typedef struct
{
    uint8_t  game_type : 4;
    uint8_t  game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t sync_timestamp;
} referee_game_state_t;

/* 0x0002 比赛结果 */
typedef struct
{
    uint8_t winner;
} referee_game_result_t;

/* 0x0003 己方机器人血量 */
typedef struct
{
    uint16_t ally_1_robot_hp;       /* 英雄 */
    uint16_t ally_2_robot_hp;       /* 工程 */
    uint16_t ally_3_robot_hp;       /* 步兵 3 */
    uint16_t ally_4_robot_hp;       /* 步兵 4 */
    uint16_t reserved;
    uint16_t ally_7_robot_hp;       /* 哨兵 */
    uint16_t ally_outpost_hp;       /* 前哨站 */
    uint16_t ally_base_hp;          /* 基地 */
} referee_game_robot_hp_t;

/* 0x0101 场地事件 */
typedef struct
{
    uint32_t event_data;
} referee_event_data_t;

/* 0x0104 裁判警告 */
typedef struct
{
    uint8_t level;                  /* 判罚等级 */
    uint8_t offending_robot_id;     /* 违规机器人 ID */
    uint8_t offense_count;          /* 违规次数 */
} referee_warning_t;

/* 0x0105 飞镖发射相关 */
typedef struct
{
    uint8_t  dart_remaining_time;
    uint16_t dart_info;             /* 位域详见协议 */
} referee_dart_info_t;

/* 0x0201 机器人性能体系 */
typedef struct
{
    uint8_t  robot_id;
    uint8_t  robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_barrel_cooling_value;      /* 枪口热量每秒冷却值 */
    uint16_t shooter_barrel_heat_limit;         /* 枪口热量上限 */
    uint16_t chassis_power_limit;               /* 底盘功率上限 */
    uint8_t  power_management_gimbal_output : 1;
    uint8_t  power_management_chassis_output : 1;
    uint8_t  power_management_shooter_output : 1;
} referee_robot_performance_t;

/* 0x0202 实时功率热量 */
typedef struct
{
    uint16_t reserved0;
    uint16_t reserved1;
    float    reserved2;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} referee_power_heat_t;

/* 0x0203 机器人位置 */
typedef struct
{
    float x;
    float y;
    float angle;
} referee_robot_position_t;

/* 0x0204 机器人增益和底盘能量 */
typedef struct
{
    uint8_t  recovery_buff;
    uint16_t cooling_buff;
    uint8_t  defence_buff;
    uint8_t  vulnerability_buff;
    uint16_t attack_buff;
    uint8_t  remaining_energy;      /* 位域详见协议 */
} referee_buff_t;

/* 0x0206 伤害状态 */
typedef struct
{
    uint8_t armor_id : 4;
    uint8_t hp_deduction_reason : 4;
} referee_robot_hurt_t;

/* 0x0207 实时射击 */
typedef struct
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float   bullet_speed;
} referee_shoot_data_t;

/* 0x0208 允许发弹量 */
typedef struct
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;     /* 堡垒提供的储备（协议长度 6 时没有这一项） */
} referee_projectile_allowance_t;

/* 0x0209 RFID 状态 */
typedef struct
{
    uint32_t rfid_status;           /* 低 32 位 */
    uint8_t  rfid_status_2;         /* 高 8 位 */
} referee_rfid_status_t;

/* 0x020A 飞镖发射站状态 */
typedef struct
{
    uint8_t  dart_launch_opening_status;        /* 0 已开启 1 关闭 2 开启/关闭中 */
    uint8_t  reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} referee_dart_launch_status_t;

/* 0x020B 地面机器人位置 */
typedef struct
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved0;
    float reserved1;
} referee_ground_robot_position_t;

/* 0x020C 雷达标记进度 */
typedef struct
{
    uint16_t mark_progress;         /* 位域详见协议 */
} referee_radar_mark_t;

/* 0x020D 哨兵信息 */
typedef struct
{
    uint32_t sentry_info;           /* 低位 */
    uint16_t sentry_info_2;         /* 高位 */
} referee_sentry_info_t;

/* 0x020E 雷达信息 */
typedef struct
{
    uint8_t radar_info;             /* 位域详见协议 */
} referee_radar_info_t;

/* 0x0303 选手端小地图下发的指令 */
typedef struct
{
    float    target_position_x;
    float    target_position_y;
    uint8_t  cmd_keyboard;
    uint8_t  target_robot_id;
    uint16_t cmd_source;
} referee_map_command_t;

/* ---------------- 0x0301 机器人交互 ---------------- */
/* 交互数据头：content_id 说明后面是什么（UI 删除/绘图、哨兵交互、自定义数据）。
 * 整个 0x0301 命令上行频率上限 10 Hz，UI 和机器人间通信共用这个带宽 */
typedef struct
{
    uint16_t data_command_id;
    uint16_t sender_id;
    uint16_t receiver_id;
} referee_interaction_header_t;

#define REFEREE_INTERACTION_HEADER_LENGTH   6u

typedef enum
{
    REFEREE_INTERACTION_UI_DELETE       = 0x0100,
    REFEREE_INTERACTION_UI_DRAW_1       = 0x0101,   /* 一次画 1 个图形 */
    REFEREE_INTERACTION_UI_DRAW_2       = 0x0102,
    REFEREE_INTERACTION_UI_DRAW_5       = 0x0103,
    REFEREE_INTERACTION_UI_DRAW_7       = 0x0104,
    REFEREE_INTERACTION_UI_DRAW_STRING  = 0x0110,
    REFEREE_INTERACTION_SENTRY_MESSAGE  = 0x0120,   /* 哨兵交互，内容 4 字节 */
    REFEREE_INTERACTION_CUSTOM_DATA     = 0x0200,   /* 0x0200 ~ 0x02FF 机器人间自定义数据 */
} referee_interaction_id_e;

#define REFEREE_UI_DELETE_LENGTH            2u
#define REFEREE_UI_GRAPH_LENGTH             15u
#define REFEREE_UI_STRING_LENGTH            (REFEREE_UI_GRAPH_LENGTH + 30u)
#define REFEREE_SENTRY_MESSAGE_LENGTH       4u

/* 收到的机器人交互：头 + 内容（内容不足 REFEREE_INTERACTION_DATA_SIZE 的部分补 0） */
typedef struct
{
    referee_interaction_header_t header;
    uint8_t                      data[REFEREE_INTERACTION_DATA_SIZE];
} referee_robot_interaction_t;

/* ---------------- UI 图形数据 ---------------- */
typedef struct
{
    uint8_t  graphic_name[3];
    uint32_t operate_type : 3;
    uint32_t graphic_type : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11;
} referee_ui_graph_t;

/* 字符串：图形头 + 30 字节字符内容 */
typedef struct
{
    referee_ui_graph_t graph;
    uint8_t            text[30];
} referee_ui_string_t;

#pragma pack(pop)

/* ---------------- UI 参数取值 ---------------- */
typedef enum
{
    REFEREE_UI_DELETE_NONE  = 0,
    REFEREE_UI_DELETE_LAYER = 1,
    REFEREE_UI_DELETE_ALL   = 2,        /* 删除全部图层，层号参数无意义 */
} referee_ui_delete_e;

typedef enum
{
    REFEREE_UI_OPERATE_ADD    = 1,
    REFEREE_UI_OPERATE_CHANGE = 2,
    REFEREE_UI_OPERATE_DELETE = 3,
} referee_ui_operate_e;

typedef enum
{
    REFEREE_UI_GRAPH_LINE      = 0,
    REFEREE_UI_GRAPH_RECTANGLE = 1,
    REFEREE_UI_GRAPH_CIRCLE    = 2,
    REFEREE_UI_GRAPH_ELLIPSE   = 3,
    REFEREE_UI_GRAPH_ARC       = 4,
    REFEREE_UI_GRAPH_FLOAT     = 5,
    REFEREE_UI_GRAPH_INT       = 6,
    REFEREE_UI_GRAPH_STRING    = 7,
} referee_ui_graph_type_e;

typedef enum
{
    REFEREE_UI_COLOR_MAIN       = 0,    /* 红蓝主色（己方颜色） */
    REFEREE_UI_COLOR_YELLOW     = 1,
    REFEREE_UI_COLOR_GREEN      = 2,
    REFEREE_UI_COLOR_ORANGE     = 3,
    REFEREE_UI_COLOR_PURPLE_RED = 4,
    REFEREE_UI_COLOR_PINK       = 5,
    REFEREE_UI_COLOR_CYAN       = 6,
    REFEREE_UI_COLOR_BLACK      = 7,
    REFEREE_UI_COLOR_WHITE      = 8,
} referee_ui_color_e;

/* ---------------- 机器人 ID ---------------- */
typedef enum
{
    REFEREE_COLOR_RED  = 0,
    REFEREE_COLOR_BLUE = 1,
} referee_color_e;

typedef enum
{
    REFEREE_ROBOT_RED_HERO       = 1,
    REFEREE_ROBOT_RED_ENGINEER   = 2,
    REFEREE_ROBOT_RED_STANDARD_1 = 3,
    REFEREE_ROBOT_RED_STANDARD_2 = 4,
    REFEREE_ROBOT_RED_STANDARD_3 = 5,
    REFEREE_ROBOT_RED_AERIAL     = 6,
    REFEREE_ROBOT_RED_SENTRY     = 7,
    REFEREE_ROBOT_RED_RADAR      = 9,
    REFEREE_ROBOT_BLUE_HERO       = 101,
    REFEREE_ROBOT_BLUE_ENGINEER   = 102,
    REFEREE_ROBOT_BLUE_STANDARD_1 = 103,
    REFEREE_ROBOT_BLUE_STANDARD_2 = 104,
    REFEREE_ROBOT_BLUE_STANDARD_3 = 105,
    REFEREE_ROBOT_BLUE_AERIAL     = 106,
    REFEREE_ROBOT_BLUE_SENTRY     = 107,
    REFEREE_ROBOT_BLUE_RADAR      = 109,
} referee_robot_id_e;

/* 选手端（客户端）ID = 0x0100 + 机器人 ID */
#define REFEREE_CLIENT_ID_BASE              0x0100u
#endif
