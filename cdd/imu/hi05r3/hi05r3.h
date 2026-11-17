#ifndef CDD_HI05R3_H__
#define CDD_HI05R3_H__
#include "component/common/types.h"

/* HI05R3（HiPNUC）驱动：经典 CAN，1 Mbit/s，J1939 扩展帧。
 * 总线、节点地址、输出频率、姿态来源在当前兵种的 ins_config.h。设备姿态模式需要模块周期发送加速度、角速度和四元数。
 * 一条报文只有一个量。设备姿态模式在加速度、角速度和四元数各收到一帧后交给回调（CAN 接收中断里调用）。 */

typedef struct
{
    float    gyro_rad_s[3];
    float    accel_m_s2[3];
    float    quat[4];               /* [w, x, y, z]，设备原始比例值；INS 任务归一化 */
    float    inclinometer_x_rad;    /* 可选 PGN 0xFF4A 安装倾角 */
    float    temperature_c;
    uint32_t timestamp_us;          /* 角速度报文到达时的 bsp_tick_us 低 32 位 */
    uint32_t communication_time_us; /* 本次加速度、角速度、四元数首帧到末帧的接收跨度 */
    uint32_t sequence;              /* 角速度帧计数，不连续说明有整组样本未发布 */
    bool     accel_fresh;           /* 加速度和这帧角速度属于同一个输出周期 */
    bool     attitude_valid;        /* 四元数与加速度属于这个输出周期，且模长合理 */
    bool     quat_from_device;      /* 四元数来自模块，而不是欧拉角换算 */
    bool     inclinometer_valid;
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
    uint32_t inclinometer_count;
    uint32_t temperature_count;
    uint32_t publish_count;
    uint32_t ignored_count;         /* 范围内但不是本驱动要的报文，或长度不对 */
} hi05r3_stats_t;

typedef void (*hi05r3_frame_callback_t)(const hi05r3_frame_t *frame, void *context);

void    hi05r3_set_frame_callback(hi05r3_frame_callback_t callback, void *context); /* 完整样本回调在 CAN 中断里调用 */
int32_t hi05r3_set_register(uint16_t address, uint16_t value);
int32_t hi05r3_set_read_register(uint16_t address, uint16_t *value);
void    hi05r3_get_stats(hi05r3_stats_t *stats);
#endif
