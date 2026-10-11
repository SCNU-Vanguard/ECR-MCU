#ifndef CDD_DT7_H__
#define CDD_DT7_H__
#include "component/common/types.h"

typedef struct {
    int16_t ch[4];          /* 右横 右竖 左横 左竖，已减中值，范围 -660..660 */
    int16_t wheel;          /* 拨轮，已减中值 */
    uint8_t s1, s2;         /* 1=上 3=中 2=下；0 = 未知（离线时） */
    int16_t mouse_x, mouse_y, mouse_z;
    uint8_t mouse_l, mouse_r;
    uint16_t key;           /* 键盘位图 */
    bool    online;         /* false = 100 ms 没收到合法帧（由 dt7_task 的事件超时判定）；此时上面所有控制量都是 0 */
    uint32_t frames, bad_frames;
} dt7_data_t;

void dt7_get(dt7_data_t *out);                      /* 取一份一致的快照（任务里调用） */
/* 解析一帧 18 字节；合法返回 true。独立成函数，方便不接硬件时做主机单元测试 */
bool dt7_parse(const uint8_t *p, dt7_data_t *out);
#endif
