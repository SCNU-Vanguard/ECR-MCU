#ifndef CDD_INTERBOARD_H__
#define CDD_INTERBOARD_H__
#include "component/common/types.h"

/* 云台（MASTER）与底盘（SLAVE）的 RS485。USART2 硬件 DE，协议里不翻转收发脚。
 * 只有一问一答：云台按 INTERBOARD_TX_HZ 发出请求缓存，底盘回同一序号的应答缓存。底盘不主动发。
 * 没有单独的应用任务。请求和应答缓存在各业务任务里填写，和上位机一样：先 load 整帧，只改自己的字段，再 store 整帧。
 * 单板兵种不要打开 CONFIG_CDD_INTERBOARD。
 * 本板是云台还是底盘，在当前兵种的 interboard_config.h 里写 INTERBOARD_ROLE。
 *
 * 帧（小端）：0xA5 | 载荷长度 | 序号 | 载荷 | CRC16 低字节 | CRC16 高字节
 * CRC16 与裁判系统相同。 */
#define INTERBOARD_ROLE_MASTER  0
#define INTERBOARD_ROLE_SLAVE   1
#include "interboard_config.h"
#ifndef INTERBOARD_ROLE
#error "interboard_config.h must define INTERBOARD_ROLE"
#endif
#if INTERBOARD_ROLE != INTERBOARD_ROLE_MASTER && INTERBOARD_ROLE != INTERBOARD_ROLE_SLAVE
#error "INTERBOARD_ROLE must be INTERBOARD_ROLE_MASTER or INTERBOARD_ROLE_SLAVE"
#endif
#if INTERBOARD_ROLE == INTERBOARD_ROLE_MASTER && !defined(INTERBOARD_TX_HZ)
#error "interboard_config.h must define INTERBOARD_TX_HZ for MASTER"
#endif

typedef struct
{
    bool     online;       /* 云台：连续 3 次等不到回答则离线。底盘：OS_TIMEOUT_50MS 没有合法请求则离线 */
    uint32_t frames;       /* 云台：对上序号且长度符合的回答。底盘：收下并回答的请求 */
    uint32_t bad_frames;   /* CRC 不对，或长度不对 */
    uint32_t timeouts;     /* 云台没等到回答；底盘从在线掉到离线的次数 */
} interboard_link_t;

void interboard_get_link(interboard_link_t *out);

#if INTERBOARD_ROLE == INTERBOARD_ROLE_MASTER
void interboard_load_request(interboard_request_t *request);
void interboard_store_request(const interboard_request_t *request);
/* 还没收到应答时返回 false。 */
bool interboard_load_reply(interboard_reply_t *reply);
#elif INTERBOARD_ROLE == INTERBOARD_ROLE_SLAVE
/* 还没收到请求时返回 false。 */
bool interboard_load_request(interboard_request_t *request);
void interboard_load_reply(interboard_reply_t *reply);
void interboard_store_reply(const interboard_reply_t *reply);
#endif
#endif
