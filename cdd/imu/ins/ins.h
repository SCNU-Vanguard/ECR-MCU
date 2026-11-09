#ifndef CDD_IMU_INS_H__
#define CDD_IMU_INS_H__
#include "component/common/types.h"
#include "component/ahrs/ahrs.h"

/* INS 快照：传感器数据每帧更新，姿态仅在解算成功时更新；valid 只表示姿态可用。 */
typedef struct
{
    bool  valid;                    /* 算法已初始化并至少推进过一次 */
    bool  temperature_valid;        /* BMI088 至少成功读取过一次温度 */
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
    uint32_t ins_cost_time;         /* 微秒：最近成功帧在任务内取原始数据前 -> 姿态输出完成。
                                    * 含快照、换算/标定、检查和解算；不含此前 DRDY/DMA/排队等待。
                                    * 区间内中断/抢占仍计入；未成功输出时保留上一有效值。 */
} ins_attitude_t;

void ins_get_attitude(ins_attitude_t *attitude);
#endif
