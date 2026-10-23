#ifndef CDD_INTERBOARD_H__
#define CDD_INTERBOARD_H__
#include "component/common/types.h"

/* 云台（MASTER）与底盘（SLAVE）的 RS485。USART2 硬件 DE，协议里不翻转收发脚。
 * 只有一问一答：云台发请求，底盘回同一序号的应答。底盘不主动发。
 * 请求和应答由若干块拼成，块宏在兵种的 interboard_config.h。
 * 单板兵种不要打开 CONFIG_CDD_INTERBOARD。
 *
 * 帧（小端）：0xA5 | 载荷长度 | 命令 | 序号 | 载荷 | CRC16 低字节 | CRC16 高字节
 * 命令固定为 1。CRC16 与裁判系统相同。 */

#define INTERBOARD_CMD  1u

/* kind：0 请求块，1 应答块。由 INTERBOARD_MOTION_REQ 等宏生成，调用处不要自己填。 */
typedef struct
{
    uint16_t offset;
    uint16_t size;
    uint8_t  kind;
} interboard_block_t;

typedef struct
{
    bool     online;       /* 云台：连续 3 次等不到回答则离线。底盘：50 ms 没有合法请求则离线 */
    uint32_t frames;       /* 云台：对上序号且长度符合的回答。底盘：收下并回答的请求 */
    uint32_t bad_frames;   /* CRC 不对，或长度不对 */
    uint32_t timeouts;     /* 云台没等到回答；底盘从在线掉到离线的次数 */
} interboard_link_t;

void interboard_get_link(interboard_link_t *out);

/* 云台只能填请求块，底盘只能填应答块。块必须由宏给出。 */
int32_t interboard_fill(interboard_block_t block, const void *data);
/* 云台取应答块，底盘取请求块。还没收到过这一侧的帧返回 false */
bool    interboard_take(interboard_block_t block, void *data);

#if defined(CONFIG_CDD_INTERBOARD_MASTER)
/* 把当前请求缓存整帧发出。没在等应答时马上发；还在等时，应答到了或超时后再发最新的缓存 */
int32_t interboard_post(void);
#endif
#endif
