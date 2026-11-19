#ifndef INTERBOARD_CONFIG_H__
#define INTERBOARD_CONFIG_H__
#include "component/common/types.h"
#include "osal/osal.h"
/* 云台发起请求。底盘板写 INTERBOARD_ROLE_SLAVE。 */
#define INTERBOARD_ROLE         INTERBOARD_ROLE_MASTER
#define INTERBOARD_PAYLOAD_MAX  64u
/* 填 OS_FREQ_xxHZ。低于 OS_FREQ_50HZ 时底盘会按 OS_TIMEOUT_50MS 反复掉线。 */
#define INTERBOARD_TX_HZ  OS_FREQ_100HZ
typedef struct BYTE_PACKED
{
    uint8_t unused;
} interboard_request_t;
typedef struct BYTE_PACKED
{
    uint8_t unused;
} interboard_reply_t;
#endif
