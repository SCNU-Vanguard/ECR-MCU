#ifndef INTERBOARD_CONFIG_H__
#define INTERBOARD_CONFIG_H__
#include "component/common/types.h"
/* 底盘只应答。云台板写 INTERBOARD_ROLE_MASTER。 */
#define INTERBOARD_ROLE         INTERBOARD_ROLE_SLAVE
#define INTERBOARD_PAYLOAD_MAX  64u
typedef struct BYTE_PACKED
{
    uint8_t unused;
} interboard_request_t;
typedef struct BYTE_PACKED
{
    uint8_t unused;
} interboard_reply_t;
#endif
