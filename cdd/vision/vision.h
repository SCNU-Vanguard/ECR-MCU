#ifndef CDD_VISION_H__
#define CDD_VISION_H__
#include "component/common/types.h"

/* USB CDC 上的视觉链路。一帧可能被拆开，也可能多帧粘在一次接收里，所以在任务里组帧。
 *
 * 帧格式（小端）：
 *   0xA5 | 载荷长度 | 命令 | 载荷 | CRC16 低字节 | CRC16 高字节
 * CRC16 覆盖前面所有字节，算法和裁判系统相同。
 * 命令号、载荷结构和载荷上限由兵种配置。收到合法帧后按命令号和长度写入对应缓存。
 * 200 ms 没有合法帧视为上位机离线。 */

#define VISION_SOF            0xA5u
#define VISION_CMD_MAX     8u

typedef struct
{
    uint8_t command;
    uint8_t length;
    void   *cache;
} vision_packet_t;

typedef struct
{
    bool     online;
    uint32_t frames;        /* 命令号和长度都对上、已写入缓存的帧 */
    uint32_t bad_frames;    /* CRC 不对，或命令号/长度没有注册项 */
    uint32_t timeouts;
} vision_link_t;

/* table 必须一直有效。重复命令号、长度超过上限、长度为 0 却没有缓存：返回 ERR_INVAL */
int32_t vision_bind(const vision_packet_t *table, uint8_t count);
/* 长度必须等于该命令注册的长度。拷进缓存再发出。未枚举返回 ERR_NODEV */
int32_t vision_write(uint8_t command, const void *payload, uint8_t length);
/* 拷贝该命令最近一次收进缓存的载荷。还没收到过，或 length 与注册长度不同：返回 false */
bool    vision_read(uint8_t command, void *payload, uint8_t length);
void    vision_get_link(vision_link_t *out);
#endif
