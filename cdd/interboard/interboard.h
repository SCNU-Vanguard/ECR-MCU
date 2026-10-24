#ifndef CDD_INTERBOARD_H__
#define CDD_INTERBOARD_H__
#include "component/common/types.h"

/* 云台（MASTER）与底盘（SLAVE）的 RS485。USART2 硬件 DE，协议里不翻转收发脚。
 * 只有一问一答：云台发整帧请求，底盘回同一序号的整帧应答。底盘不主动发。
 * 帧布局在兵种的 interboard_config.h。单板兵种不要打开 CONFIG_CDD_INTERBOARD。
 * 要改本侧缓存时，先 load 整帧，只改自己的字段，再 store 整帧。
 *
 * 帧（小端）：0xA5 | 载荷长度 | 命令 | 序号 | 载荷 | CRC16 低字节 | CRC16 高字节
 * 命令固定为 1。CRC16 与裁判系统相同。 */
#include "interboard_config.h"

#define INTERBOARD_CMD  1u

typedef struct
{
    bool     online;       /* 云台：连续 3 次等不到回答则离线。底盘：50 ms 没有合法请求则离线 */
    uint32_t frames;       /* 云台：对上序号且长度符合的回答。底盘：收下并回答的请求 */
    uint32_t bad_frames;   /* CRC 不对，或长度不对 */
    uint32_t timeouts;     /* 云台没等到回答；底盘从在线掉到离线的次数 */
} interboard_link_t;

void interboard_get_link(interboard_link_t *out);

#if defined(CONFIG_CDD_INTERBOARD_MASTER)
/* 待发请求。load 一定把当前整帧拷出来。 */
void interboard_load_request(interboard_request_t *request);
void interboard_store_request(const interboard_request_t *request);
/* 最近一次应答。还没收到返回 false。 */
bool interboard_load_reply(interboard_reply_t *reply);
#elif defined(CONFIG_CDD_INTERBOARD_SLAVE)
/* 最近一次请求。还没收到返回 false。 */
bool interboard_load_request(interboard_request_t *request);
/* 待发应答。load 一定把当前整帧拷出来。 */
void interboard_load_reply(interboard_reply_t *reply);
void interboard_store_reply(const interboard_reply_t *reply);
#endif

#if defined(CONFIG_CDD_INTERBOARD_MASTER)
/* 把当前请求缓存整帧发出。没在等应答时马上发；还在等时，应答到了或超时后再发最新的缓存 */
int32_t interboard_post(void);
#endif
#endif
