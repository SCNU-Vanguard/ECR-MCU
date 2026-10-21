#ifndef INTERBOARD_CONFIG_H__
#define INTERBOARD_CONFIG_H__
#include "component/common/types.h"

/* 板间命令。外层帧和问答在 cdd/interboard，载荷按 1 字节 packed。
 * 请求和应答可以是不同的结构体。长度为 0 的空结构体表示这一侧没有载荷。
 * 长度字段是 1 字节，所以上限不能超过 255。
 * INTERBOARD_CMD_TABLE 在 cdd/interboard 里展开成静态缓存并注册。 */

#define INTERBOARD_PAYLOAD_MAX  32u

#define INTERBOARD_CMD_CONTROL  1u
#define INTERBOARD_CMD_MOTION   2u
#define INTERBOARD_CMD_REFEREE  3u

typedef struct BYTE_PACKED
{
} interboard_empty_t;

typedef struct BYTE_PACKED
{
    float forward_m_s;
    float right_m_s;
    float spin_rad_s;
    float yaw_rad;
} interboard_control_t;

typedef struct BYTE_PACKED
{
    uint8_t accepted;   /* 0：底盘收下 */
} interboard_control_ack_t;

typedef struct BYTE_PACKED
{
    float forward_m_s;
    float right_m_s;
    float spin_rad_s;
} interboard_motion_t;

typedef struct BYTE_PACKED
{
    uint16_t hp;
    uint16_t bullet_allowance;
    uint16_t power_limit_w;
} interboard_referee_t;

#define INTERBOARD_CMD_TABLE(X)                                                         \
    X(CONTROL, INTERBOARD_CMD_CONTROL, interboard_control_t, interboard_control_ack_t)  \
    X(MOTION,  INTERBOARD_CMD_MOTION,  interboard_empty_t,   interboard_motion_t)       \
    X(REFEREE, INTERBOARD_CMD_REFEREE, interboard_empty_t,   interboard_referee_t)

_Static_assert(sizeof(interboard_empty_t) == 0u, "empty request must occupy no bytes");
_Static_assert(sizeof(interboard_control_t) <= INTERBOARD_PAYLOAD_MAX, "control exceeds interboard payload");
_Static_assert(sizeof(interboard_control_ack_t) <= INTERBOARD_PAYLOAD_MAX, "control ack exceeds interboard payload");
_Static_assert(sizeof(interboard_motion_t) <= INTERBOARD_PAYLOAD_MAX, "motion exceeds interboard payload");
_Static_assert(sizeof(interboard_referee_t) <= INTERBOARD_PAYLOAD_MAX, "referee exceeds interboard payload");
#endif
