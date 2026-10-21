#ifndef VISION_CONFIG_H__
#define VISION_CONFIG_H__
#include "component/common/types.h"

/* 视觉包。外层帧在 cdd/vision，载荷按 1 字节 packed。每种载荷都不能超过 VISION_PAYLOAD_MAX。
 * 长度字段是 1 字节，所以上限不能超过 255。
 * VISION_PACKET_TABLE 在 cdd/vision 里展开成静态缓存并注册。 */

#define VISION_PAYLOAD_MAX  32u

#define VISION_CMD_ATTITUDE  0x01u   /* 板 -> 上位机：当前 roll、pitch、yaw */
#define VISION_CMD_TARGET    0x02u   /* 上位机 -> 板：目标 roll、pitch、yaw */

typedef struct BYTE_PACKED
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} vision_attitude_t;

typedef struct BYTE_PACKED
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    uint8_t is_locked;
    uint8_t shoot_en;
} vision_target_t;

#define VISION_PACKET_TABLE(X)                                 \
    X(ATTITUDE, VISION_CMD_ATTITUDE, vision_attitude_t)        \
    X(TARGET,   VISION_CMD_TARGET,   vision_target_t)

_Static_assert(sizeof(vision_attitude_t) <= VISION_PAYLOAD_MAX, "attitude exceeds vision payload");
_Static_assert(sizeof(vision_target_t) <= VISION_PAYLOAD_MAX, "target exceeds vision payload");
#endif
