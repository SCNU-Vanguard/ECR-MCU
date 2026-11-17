#ifndef CDD_IMU_INS_H__
#define CDD_IMU_INS_H__
#include "component/common/types.h"
#include "component/ahrs/ahrs.h"

/* INS 快照：传感器数据每帧更新，姿态仅在解算成功时更新；valid 只表示姿态可用。 */
typedef struct
{
    bool  valid;                    /* 算法已初始化并至少推进过一次 */
    bool  temperature_valid;        /* 至少成功读取过一次温度 */
    float quaternion[4];            /* [w, x, y, z]，机体系 -> 固定参考坐标系 */
    float roll_rad;
    float pitch_rad;
    float yaw_rad;                  /* (-pi, pi] */
    float yaw_total_rad;            /* 连续累计 yaw */
    float gyro_rad_s[3];            /* 标定后的角速度（机体系） */
    float accel_m_s2[3];            /* 标定后的加速度（机体系，最近一帧） */
    float inclinometer_x_rad;       /* HI05R3 可选 X 轴倾角 */
    bool  inclinometer_valid;
    float temperature_c;
    uint32_t imu_frame_count;       /* 已发布的陀螺仪帧数，独立于姿态更新 */
    uint32_t update_count;
    uint32_t ins_cost_time;         /* 微秒：HI05R3 设备姿态为三类 CAN 帧首帧到末帧的接收跨度；
                                    * 其余路径为任务取原始数据前到姿态输出完成的耗时。 */
} ins_attitude_t;

void ins_get_attitude(ins_attitude_t *attitude);
#endif
