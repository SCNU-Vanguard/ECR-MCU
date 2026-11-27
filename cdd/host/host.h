#ifndef CDD_HOST_H__
#define CDD_HOST_H__
#include "component/common/types.h"

/* USB CDC 上的上位机链路。各业务共用这一条，用命令号区分。
 * 一帧可能被拆开，也可能多帧粘在一次接收里，所以在任务里组帧。
 *
 * 帧格式（小端），方向不在帧里，而在命令表的 HOST_SEND / HOST_RECV：
 *   0xA5 | 载荷长度 | 命令 | 载荷 | CRC16 低字节 | CRC16 高字节
 * CRC16 覆盖前面所有字节：正向多项式 0x1021，初值 0xFFFF，低位先入，低字节在前。
 * 收到合法帧后，只有 HOST_RECV 的命令会写入缓存。HOST_SEND 或没注册的命令算坏帧。
 * 200 ms 没有合法帧视为上位机离线。板按 HOST_TX_HZ 把 HOST_SEND 缓存发出去。 */

enum
{
    HOST_SEND = 0, /* 板 → 上位机。host_store 写入待发缓存。收到这个命令算坏帧 */
    HOST_RECV = 1, /* 上位机 → 板。host_load 读最近一次缓存 */
};

typedef struct
{
    bool     online;
    uint32_t frames;        /* 命令号、方向和长度都对上、已写入缓存的帧 */
    uint32_t bad_frames;    /* CRC 不对，或命令不该由上位机发来、长度不对 */
    uint32_t timeouts;
    uint32_t tx_frames;     /* 已写入 USB 的发送帧 */
    uint32_t tx_fail;       /* 队列满，没写进去 */
} host_link_t;

/* 按命令写入 HOST_SEND 缓存。命令未注册、方向或长度不对时不写。 */
void host_store(uint8_t command, const void *payload, uint8_t length);
/* 按命令读出 HOST_RECV 缓存。还没收到、命令或长度不对时返回 false。 */
bool host_load(uint8_t command, void *payload, uint8_t length);
void host_get_link(host_link_t *out);
#endif
