#ifndef CDD_INTERBOARD_H__
#define CDD_INTERBOARD_H__
#include "component/common/types.h"

/* 云台（MASTER）与底盘（SLAVE）的 RS485。USART2 硬件 DE，协议里不翻转收发脚。
 * 云台先发一帧，底盘收齐后回同一命令、同一序号；底盘不主动发。
 * 命令由兵种用 interboard_cmd_t 注册。本模块按命令号和长度把载荷写入对应缓存。
 * 单板兵种不要打开 CONFIG_CDD_INTERBOARD。
 *
 * 帧（小端）：0xA5 | 载荷长度 | 命令 | 序号 | 载荷 | CRC16 低字节 | CRC16 高字节
 * CRC16 与裁判系统相同。载荷上限由兵种配置。 */

#define INTERBOARD_CMD_MAX      8u

typedef struct
{
    uint8_t command;
    uint8_t request_length;   /* 0：这一侧没有载荷，request 为 NULL */
    uint8_t reply_length;
    void   *request;
    void   *reply;
} interboard_cmd_t;

typedef struct
{
    bool     online;       /* 云台：连续 3 次等不到回答则离线。底盘：50 ms 没有合法请求则离线 */
    uint32_t frames;       /* 云台：对上序号且长度符合的回答。底盘：收下并回答的请求 */
    uint32_t bad_frames;   /* CRC 不对，或命令号/长度没有注册项 */
    uint32_t timeouts;     /* 云台没等到回答；底盘从在线掉到离线的次数 */
} interboard_link_t;

/* table 必须一直有效。重复命令号或长度和缓存对不上：返回 ERR_INVAL */
int32_t interboard_bind(const interboard_cmd_t *table, uint8_t count);
void    interboard_get_link(interboard_link_t *out);

#if defined(CONFIG_CDD_INTERBOARD_MASTER)
/* 长度必须等于该命令的 request_length。没有在等应答时马上发出；还在等时，后一次 post 覆盖上一帧 */
int32_t interboard_post(uint8_t command, const void *payload, uint8_t length);
/* 拷贝该命令最近一次应答。还没收到过，或 length 与 reply_length 不同：返回 false */
bool    interboard_read(uint8_t command, void *payload, uint8_t length);
#elif defined(CONFIG_CDD_INTERBOARD_SLAVE)
/* 长度必须等于该命令的 reply_length。收到请求时回这份缓存 */
int32_t interboard_write(uint8_t command, const void *payload, uint8_t length);
/* 拷贝该命令最近一次请求。还没收到过，或 length 与 request_length 不同：返回 false */
bool    interboard_read(uint8_t command, void *payload, uint8_t length);
#endif
#endif
