#ifndef INTERBOARD_CONFIG_H__
#define INTERBOARD_CONFIG_H__
#include "component/common/types.h"
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
