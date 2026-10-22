#ifndef CDD_HOST_H__
#define CDD_HOST_H__
#include "component/common/types.h"

/* USB CDC 上的上位机链路。视觉和导航共用这一条，用命令号区分。
 * 一帧可能被拆开，也可能多帧粘在一次接收里，所以在任务里组帧。
 *
 * 帧格式（小端），方向不在帧里，而在命令表的 HOST_SEND / HOST_RECV：
 *   0xA5 | 载荷长度 | 命令 | 载荷 | CRC16 低字节 | CRC16 高字节
 * CRC16 覆盖前面所有字节，算法和裁判系统相同。
 * 收到合法帧后，只有 HOST_RECV 的命令会写入缓存。HOST_SEND 或没注册的命令算坏帧。
 * 200 ms 没有合法帧视为上位机离线。 */

#define HOST_SOF        0xA5u
#define HOST_CMD_MAX    8u

enum
{
    HOST_SEND = 0, /* 板 → 上位机。收到这个命令算坏帧 */
    HOST_RECV = 1, /* 上位机 → 板。host_transfer 读最近一次缓存 */
};

typedef struct
{
    uint8_t command;
    uint8_t direction;
    uint8_t length;
    void   *cache;
} host_packet_t;

typedef struct
{
    bool     online;
    uint32_t frames;        /* 命令号、方向和长度都对上、已写入缓存的帧 */
    uint32_t bad_frames;    /* CRC 不对，或命令不该由上位机发来、长度不对 */
    uint32_t timeouts;
} host_link_t;

/* table 必须一直有效。重复命令号、方向非法、长度和缓存对不上：返回 ERR_INVAL */
int32_t host_bind(const host_packet_t *table, uint8_t count);
/* HOST_SEND：组帧发出。HOST_RECV：拷出最近一次收到的载荷。
 * 没注册、方向对不上调用方式、长度不对：ERR_INVAL。
 * 发送时未枚举 ERR_NODEV，队列满 ERR_BUSY。还没收到过：ERR_TIMEOUT */
int32_t host_transfer(uint8_t command, void *payload, uint8_t length);
void    host_get_link(host_link_t *out);
#endif
