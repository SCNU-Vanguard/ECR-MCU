#ifndef VISION_CONFIG_H__
#define VISION_CONFIG_H__
#include "component/common/types.h"

/* demo 视觉载荷。外层帧见 cdd/vision/vision.h。三个角都是小端 float，单位 rad。 */
#define VISION_CMD_ATTITUDE  0x01u   /* 板 -> 上位机：当前 roll、pitch、yaw */
#define VISION_CMD_TARGET    0x02u   /* 上位机 -> 板：目标 roll、pitch、yaw */

typedef struct
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} vision_angles_t;
#endif
