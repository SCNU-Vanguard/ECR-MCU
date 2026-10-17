#ifndef CDD_IMU_INS_H__
#define CDD_IMU_INS_H__
#include "component/common/types.h"
#include "component/ahrs/ahrs.h"

/* 姿态输出，由 ins_task 在每个 IMU 样本之后更新；其他任务用 ins_get_attitude 读取一致的快照 */
typedef struct
{
    bool  valid;                    /* 算法已初始化并至少推进过一次 */
    float quaternion[4];            /* [w, x, y, z]，机体系 -> 导航系 */
    float roll_rad;
    float pitch_rad;
    float yaw_rad;                  /* (-pi, pi] */
    float yaw_total_rad;            /* 连续累计 yaw */
    float gyro_rad_s[3];            /* 标定后的角速度（机体系） */
    float temperature_c;
    uint32_t update_count;
} ins_attitude_t;

void ins_get_attitude(ins_attitude_t *attitude);
#endif
