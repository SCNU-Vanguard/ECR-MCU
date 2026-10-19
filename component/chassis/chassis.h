#ifndef COMPONENT_CHASSIS_H__
#define COMPONENT_CHASSIS_H__
#include "component/common/types.h"

/* 解算用的底盘坐标系：x 向前，y 向右，自旋俯视逆时针为正。三个速度都是底盘中心的速度。
 * 数组下标 0–3 只按数学象限编号，不参与公式。俯视、车头朝上时，数学坐标的 +x 向右、+y 向前：
 *   1 左前（第二象限）    0 右前（第一象限）
 *   2 左后（第三象限）    3 右后（第四象限）
 * 麦轮正角速度是把底盘向前推的方向。全向轮正角速度沿该轮 45° 的驱动方向，纯前进时右侧两轮为负。
 * 电机装反时用电机自己的 reverse，不写进解算。 */
enum
{
    CHASSIS_WHEEL_FRONT_RIGHT = 0,    /* 第一象限：右前 */
    CHASSIS_WHEEL_FRONT_LEFT,         /* 第二象限：左前 */
    CHASSIS_WHEEL_REAR_LEFT,          /* 第三象限：左后 */
    CHASSIS_WHEEL_REAR_RIGHT,         /* 第四象限：右后 */
    CHASSIS_WHEEL_COUNT
};

typedef struct
{
    float forward_m_s; /* 向前为正 */
    float right_m_s;   /* 向右为正 */
    float spin_rad_s;  /* 俯视逆时针为正 */
} chassis_speed_t;
#endif
