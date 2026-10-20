#ifndef CDD_INTERBOARD_H__
#define CDD_INTERBOARD_H__
#include "component/common/types.h"

/* 云台（MASTER）与底盘（SLAVE）的 RS485。USART2 硬件 DE，协议里不翻转收发脚。
 * 云台先发一帧，底盘收齐后回同一命令、同一序号；底盘不主动发，总线上不会对撞。
 * 本模块只处理外层帧。命令号和载荷布局在兵种自己的 interboard_config.h。
 * 单板兵种不要打开 CONFIG_CDD_INTERBOARD。
 *
 * 帧（小端）：0xA5 | 载荷长度 | 命令 | 序号 | 载荷 | CRC16 低字节 | CRC16 高字节
 * CRC16 与裁判系统相同。载荷最长 INTERBOARD_PAYLOAD_MAX。 */

#define INTERBOARD_PAYLOAD_MAX  32u

typedef struct
{
    bool     online;       /* 云台：连续 3 次等不到回答则离线。底盘：50 ms 没有合法请求则离线 */
    uint32_t frames;       /* 云台：对上序号的回答。底盘：收下并尝试回答的请求 */
    uint32_t bad_frames;   /* CRC 不对，或载荷长度超过上限 */
    uint32_t timeouts;     /* 云台没等到回答；底盘从在线掉到离线的次数 */
} interboard_link_t;

void interboard_get_link(interboard_link_t *out);

#if defined(CONFIG_CDD_INTERBOARD_MASTER)
/* 后一次 post 覆盖还没发出去的上一次。正在等回答时，新的请求排到这次结束之后。
 * 载荷可以是 NULL，长度必须是 0。 */
int32_t interboard_post(uint8_t command, const void *payload, uint8_t length);
/* 按命令号拷贝最近一次对上序号的回答。还没收到过返回 false，不写 payload */
bool interboard_get_reply(uint8_t command, void *payload, uint8_t *length);
#elif defined(CONFIG_CDD_INTERBOARD_SLAVE)
/* 收到该命令时回这份载荷。每种命令保留一份，最多 8 种。 */
int32_t interboard_set_reply(uint8_t command, const void *payload, uint8_t length);
/* 按命令号拷贝最近一次请求。还没收到过返回 false，不写 payload */
bool interboard_get_request(uint8_t command, void *payload, uint8_t *length);
#endif
#endif
