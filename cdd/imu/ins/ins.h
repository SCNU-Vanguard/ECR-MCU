#ifndef CDD_IMU_INS_H__
#define CDD_IMU_INS_H__
#include "component/common/types.h"
#include "component/ahrs/ahrs.h"

/* 数据源。一个兵种只选一个，取值是 ins_config.h 的 INS_IMU。
 * 没选中的 IMU 参数不参与编译。驱动由 robot.config 的 CONFIG_CDD_IMU_* 决定编不编。 */
#define INS_IMU_BMI088                  0
#define INS_IMU_HI05R3                  1

/* HI05R3 姿态来源。只在选中 HI05R3 时，由 ins_config.h 的 HI05R3_ATTITUDE 指定。 */
#define HI05R3_ATTITUDE_AHRS            0   /* 角速度、加速度交给 INS_ALGORITHM */
#define HI05R3_ATTITUDE_DEVICE          1   /* 使用模块四元数 */

/* INS 快照：传感器数据每帧更新，姿态仅在解算成功时更新；valid 只表示姿态可用。 */
typedef struct
{
    bool  valid;                    /* 算法已初始化并至少推进过一次 */
    bool  temperature_valid;        /* BMI088 曾读到温度，或 HI05R3 曾收到可选温度帧；
                                     * 不表示温度仍新鲜，HI05R3 姿态更新不依赖温度。 */
    float quaternion[4];            /* [w, x, y, z]，机体系 -> 固定参考坐标系 */
    float roll_rad;
    float pitch_rad;
    float yaw_rad;                  /* (-pi, pi] */
    float yaw_total_rad;            /* 连续累计 yaw */
    float gyro_rad_s[3];            /* 标定后的角速度（机体系） */
    float accel_m_s2[3];            /* 标定后的加速度（机体系，最近一帧） */
    float temperature_c;
    uint32_t imu_frame_count;       /* 已发布的陀螺仪帧数，独立于姿态更新 */
    uint32_t update_count;
    uint32_t elapsed_us;            /* 微秒。HI05R3 设备姿态：三类 CAN 帧首帧到末帧的接收跨度。
                                     * 其余路径：任务取原始数据前到姿态输出完成的耗时。 */
} ins_attitude_t;

void ins_get_attitude(ins_attitude_t *attitude);
#endif
