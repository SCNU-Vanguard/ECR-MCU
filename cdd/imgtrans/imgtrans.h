#ifndef CDD_IMGTRANS_H__
#define CDD_IMGTRANS_H__
#include "component/common/types.h"

/* VT03 图传链路遥控器（UART_IMGTRANS，只收，21 字节一帧，帧头 0xA9 0x53，整帧 CRC16）。
 * 和 UART_VOFA 共用 USART1，这里不发送。
 * 通道顺序和 dt7 一致：右横、右竖、左横、左竖，已减中值。
 * 离线（100 ms 没收到合法帧）时 online = false，除计数外所有控制量清零。
 * 哪个遥控器生效由应用层决定，这里不看 dt7 的在线状态。 */

typedef struct
{
    int16_t  channel[4];    /* 右横 右竖 左横 左竖，已减中值；超出 -660..660 的轴记 0 */
    int16_t  wheel;         /* 拨轮，已减中值；超范围记 0 */
    uint8_t  gear;          /* 0 = C，1 = N，2 = S */
    uint8_t  pause;
    uint8_t  custom_left;   /* 左侧自定义键 */
    uint8_t  custom_right;  /* 右侧自定义键 */
    uint8_t  trigger;       /* 扳机 */
    int16_t  mouse_x;
    int16_t  mouse_y;
    int16_t  mouse_z;
    uint8_t  mouse_left;    /* 2 位：按下状态 */
    uint8_t  mouse_right;
    uint8_t  mouse_middle;
    uint16_t key;           /* 键盘位图，位序和 dt7 相同：W S A D Shift Ctrl Q E R F G Z X C V B */
    bool     online;
    uint32_t frames;
    uint32_t bad_frames;    /* 帧头对上但 CRC16 不对的次数 */
} imgtrans_data_t;

void imgtrans_get(imgtrans_data_t *out);            /* 取一份一致的快照（任务里调用） */
/* 解析一帧 21 字节：帧头和 CRC16 都对才返回 true。独立成函数，方便不接硬件时做主机单元测试 */
bool imgtrans_parse(const uint8_t *frame, imgtrans_data_t *out);
#endif
