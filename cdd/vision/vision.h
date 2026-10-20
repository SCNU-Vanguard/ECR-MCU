#ifndef CDD_VISION_H__
#define CDD_VISION_H__
#include "component/common/types.h"

/* USB CDC 上的视觉链路。CDC 是字节流，一帧可能被拆开，也可能多帧粘在一次接收里，所以在任务里组帧。
 *
 * 帧格式（小端）：
 *   0xA5 | 载荷长度 | 命令 | 载荷 | CRC16 低字节 | CRC16 高字节
 * CRC16 覆盖前面所有字节，算法和裁判系统相同（crc16_append / crc16_verify）。
 * 载荷内容由各兵种自己定，本模块只检查外层。载荷最长 VISION_PAYLOAD_MAX。
 * 收到合法帧后调用绑定的回调。200 ms 没有合法帧视为上位机离线，回调不因此触发。 */

#define VISION_SOF            0xA5u
#define VISION_PAYLOAD_MAX    32u

typedef void (*vision_rx_fn)(uint8_t command, const uint8_t *payload, uint8_t length, void *argument);

typedef struct
{
    bool     online;        /* 最近 200 ms 内收到过 CRC 正确的帧 */
    uint32_t frames;        /* CRC 正确的帧数，含本兵种不认识的命令 */
    uint32_t bad_frames;    /* 帧头像对过、但长度或 CRC 不对 */
} vision_link_t;

/* 组一帧并从 USB 发出。未枚举时不入队，返回 ERR_NODEV。队列放不下整帧返回 ERR_BUSY */
int32_t vision_send(uint8_t command, const void *payload, uint8_t length);
void    vision_bind_rx(vision_rx_fn fn, void *argument);
void    vision_get_link(vision_link_t *out);
#endif
