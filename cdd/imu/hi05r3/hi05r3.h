#ifndef CDD_HI05R3_H__
#define CDD_HI05R3_H__
#include "component/common/types.h"

/* HI05R3（HiPNUC）驱动：经典 CAN，1 Mbit/s，J1939 扩展帧。
 * 总线、节点、输出频率和 HI05R3_ATTITUDE 在当前兵种的 ins_config.h，且只在 INS 选中 HI05R3 时定义。
 * 本驱动由 CONFIG_CDD_IMU_HI05R3 决定编不编。
 * HI05R3_ATTITUDE_DEVICE 需要模块周期发送加速度、角速度和四元数。
 * 一条报文只有一个量。设备姿态模式在加速度、角速度和四元数各收到一帧后交给回调（CAN 接收中断里调用）。 */

typedef struct
{
    float    gyro_rad_s[3];
    float    accel_m_s2[3];
    float    quat[4];               /* [w, x, y, z]，设备原始比例值；INS 任务归一化 */
    float    temperature_c;
    uint32_t timestamp_us;          /* 角速度报文到达时的 bsp_tick_us 低 32 位 */
    uint32_t communication_time_us; /* 本次加速度、角速度、四元数首帧到末帧的接收跨度 */
    uint32_t sequence;              /* 角速度帧计数，不连续说明有整组样本未发布 */
    bool     accel_fresh;           /* 加速度和这帧角速度属于同一个输出周期 */
    bool     attitude_valid;        /* 四元数与加速度属于这个输出周期，且模长合理 */
    bool     temperature_valid;
    bool     gyro_clipped;
    bool     accel_clipped;
} hi05r3_frame_t;

typedef struct
{
    uint32_t rx_frames;
    uint32_t acc_count;
    uint32_t gyro_count;
    uint32_t attitude_count;        /* 横滚/俯仰帧 */
    uint32_t quat_count;
    uint32_t inclinometer_count;
    uint32_t temperature_count;
    uint32_t publish_count;
    uint32_t ignored_count;         /* 范围内但不是本驱动要的报文，或长度不对 */
} hi05r3_stats_t;

/* J1939 主机地址，模块节点地址 HI05R3_NODE_ID 不能和它相同。 */
#define HI05R3_HOST_ADDRESS  0x55u

/* 配置寄存器 0x0000 是控制寄存器，只接受下面两个值。 */
#define HI05R3_CTRL_ADDR     0x0000u
#define HI05R3_CTRL_SAVE     0x0000u    /* 保存当前配置 */
#define HI05R3_CTRL_REBOOT   0x00FFu    /* 重启模块 */

typedef void (*hi05r3_frame_callback_t)(const hi05r3_frame_t *frame, void *context);

void    hi05r3_set_frame_callback(hi05r3_frame_callback_t callback, void *context); /* 完整样本回调在 CAN 中断里调用 */
/* 写、读模块寄存器。相邻配置帧间隔 10 ms，读寄存器最多等 100 ms，返回 ERR_TIMEOUT 表示没有响应。
 * 会阻塞调用任务，不能在中断里调用。读不支持控制寄存器 0x0000。 */
int32_t hi05r3_set_register(uint16_t address, uint16_t value);
int32_t hi05r3_set_read_register(uint16_t address, uint16_t *value);
/* 模块自己输出的欧拉角（弧度，逆时针为正）。INS 和应用层不使用，只给调试和核对四元数；
 * 横滚/俯仰和航向都有且没过期（1 s）才返回 true。 */
bool    hi05r3_get_euler(float *roll_rad, float *pitch_rad, float *yaw_rad);
/* 最近一次安装倾角（X 轴，弧度）。不随 INS 姿态发布，只给调试查看；超过 1 s 没更新返回 false。 */
bool    hi05r3_get_inclinometer(float *x_rad);
void    hi05r3_get_stats(hi05r3_stats_t *stats);
#endif
