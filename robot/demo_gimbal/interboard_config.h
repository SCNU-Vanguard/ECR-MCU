#ifndef INTERBOARD_CONFIG_H__
#define INTERBOARD_CONFIG_H__
#include "component/common/types.h"

/* 板间只有一问一答，1000 Hz。一帧请求、一帧应答，不再拆块。
 * 任务要改某一侧的缓存时，先把整帧读出来，只改自己的字段，再整帧写回去。
 * 请求里的前后、左右是云台坐标系。底盘用 yaw 电机角度旋到车体坐标。
 * 应答：车体前后、左右、自旋，yaw 电机多圈角度，17 mm 枪口当前热量和热量上限。
 * 云台板和底盘板各有一份，布局必须一致，由开发者自己对齐。 */

#define INTERBOARD_PAYLOAD_MAX  64u

typedef struct BYTE_PACKED
{
    float   forward_m_s;
    float   right_m_s;
    float   spin_rad_s;
    float   yaw_rad;
    float   yaw_target_rad;
    uint8_t enabled;
    uint8_t yaw_valid;
} interboard_motion_req_t;

typedef struct BYTE_PACKED
{
    uint8_t trigger_on;
} interboard_shoot_req_t;

typedef struct BYTE_PACKED
{
    interboard_motion_req_t motion;
    interboard_shoot_req_t  shoot;
} interboard_request_t;

typedef struct BYTE_PACKED
{
    float    forward_m_s;
    float    right_m_s;
    float    spin_rad_s;
    float    yaw_motor_rad;
    uint16_t heat;
    uint16_t heat_limit;
} interboard_reply_t;
#endif
