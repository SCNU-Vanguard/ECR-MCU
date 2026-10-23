#ifndef INTERBOARD_CONFIG_H__
#define INTERBOARD_CONFIG_H__
#include <stddef.h>
#include "component/common/types.h"
#include "cdd/interboard/interboard.h"

/* 板间只有一问一答。请求和应答各是若干块拼成的一帧。
 * 任务用 INTERBOARD_MOTION_REQ 这类宏调用 interboard_fill / interboard_take，不写偏移。
 * 云台填请求块、取应答块。底盘填应答块、取请求块。 */

#define INTERBOARD_PAYLOAD_MAX  64u

typedef struct BYTE_PACKED
{
    float forward_m_s;
    float right_m_s;
    float spin_rad_s;
    float yaw_rad;
} interboard_motion_req_t;

typedef struct BYTE_PACKED
{
    float   trigger_rad_s;
    int16_t friction_cmd;
} interboard_shoot_req_t;

typedef struct BYTE_PACKED
{
    float forward_m_s;
    float right_m_s;
    float spin_rad_s;
} interboard_motion_rep_t;

typedef struct BYTE_PACKED
{
    uint16_t hp;
    uint16_t bullet_allowance;
    uint16_t power_limit_w;
} interboard_referee_rep_t;

typedef struct BYTE_PACKED
{
    interboard_motion_req_t  motion;
    interboard_shoot_req_t   shoot;
} interboard_request_t;

typedef struct BYTE_PACKED
{
    interboard_motion_rep_t  motion;
    interboard_referee_rep_t referee;
} interboard_reply_t;

#define INTERBOARD_REQ(member) \
    ((interboard_block_t){ (uint16_t)offsetof(interboard_request_t, member), (uint16_t)sizeof(((interboard_request_t *)0)->member), 0u })
#define INTERBOARD_REP(member) \
    ((interboard_block_t){ (uint16_t)offsetof(interboard_reply_t, member), (uint16_t)sizeof(((interboard_reply_t *)0)->member), 1u })

#define INTERBOARD_MOTION_REQ   INTERBOARD_REQ(motion)
#define INTERBOARD_SHOOT_REQ    INTERBOARD_REQ(shoot)
#define INTERBOARD_MOTION_REP   INTERBOARD_REP(motion)
#define INTERBOARD_REFEREE_REP  INTERBOARD_REP(referee)
#endif
