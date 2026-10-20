#ifndef INTERBOARD_CONFIG_H__
#define INTERBOARD_CONFIG_H__
#include "component/common/types.h"
#include "cdd/interboard/interboard.h"

/* 板间载荷。外层帧和问答在 cdd/interboard。每种载荷都不能超过 INTERBOARD_PAYLOAD_MAX。 */

#define INTERBOARD_CMD_CONTROL  1u   /* 云台写下控制量；底盘回 1 字节，0 表示收下 */
#define INTERBOARD_CMD_MOTION   2u   /* 云台要运动数据；底盘回 forward / right / spin */
#define INTERBOARD_CMD_REFEREE  3u   /* 云台要裁判系统快照 */

typedef struct
{
    float forward_m_s;
    float right_m_s;
    float spin_rad_s;
    float yaw_rad;
} interboard_control_t;

typedef struct
{
    float forward_m_s;
    float right_m_s;
    float spin_rad_s;
} interboard_motion_t;

typedef struct
{
    uint16_t hp;
    uint16_t bullet_allowance;
    uint16_t power_limit_w;
} interboard_referee_t;

_Static_assert(sizeof(interboard_control_t) <= INTERBOARD_PAYLOAD_MAX, "control command exceeds interboard payload");
_Static_assert(sizeof(interboard_motion_t) <= INTERBOARD_PAYLOAD_MAX, "motion exceeds interboard payload");
_Static_assert(sizeof(interboard_referee_t) <= INTERBOARD_PAYLOAD_MAX, "referee snapshot exceeds interboard payload");
#endif
