#ifndef CDD_HI05R3_H__
#define CDD_HI05R3_H__
#include "component/common/types.h"

/* HI05R3（HiPNUC）驱动：经典 CAN，1 Mbit/s，J1939 扩展帧。
 * 总线、节点地址、输出频率、姿态来源在当前兵种的 ins_config.h。模块自己要按这个频率发加速度、角速度、横滚/俯仰和航向。
 * 一条报文只有一个量。驱动把同一输出周期里的加速度、角速度和姿态拼成一帧，在角速度报文到达时交给回调（CAN 接收中断里调用）。 */

typedef struct
{
    float    gyro_rad_s[3];
    float    accel_m_s2[3];
    float    roll_rad;
    float    pitch_rad;
    float    yaw_rad;               /* 模块欧拉角，(-pi, pi] */
    float    quat[4];               /* [w, x, y, z]。没有四元数报文时由欧拉角换算 */
    float    temperature_c;
    uint32_t timestamp_us;          /* 角速度报文到达时的 bsp_tick_us 低 32 位 */
    uint32_t sequence;              /* 从 1 开始的发布计数，不连续说明丢了周期 */
    bool     accel_fresh;           /* 加速度和这帧角速度属于同一个输出周期 */
    bool     attitude_valid;        /* 横滚/俯仰和航向都属于这个周期 */
    bool     quat_from_device;      /* 四元数来自模块，而不是欧拉角换算 */
    bool     temperature_valid;
    bool     gyro_clipped;
    bool     accel_clipped;
} hi05r3_frame_t;

typedef struct
{
    uint32_t rx_frames;
    uint32_t acc_count;
    uint32_t gyro_count;
    uint32_t attitude_count;
    uint32_t quat_count;
    uint32_t temperature_count;
    uint32_t publish_count;
    uint32_t ignored_count;         /* 范围内但不是本驱动要的报文，或长度不对 */
} hi05r3_stats_t;

typedef void (*hi05r3_frame_callback_t)(const hi05r3_frame_t *frame, void *context);

void    hi05r3_set_frame_callback(hi05r3_frame_callback_t callback, void *context); /* 回调在 CAN 中断里调用 */
int32_t hi05r3_start(void);                                                         /* 订阅本节点的数据报文 */
void    hi05r3_get_stats(hi05r3_stats_t *stats);
#endif
