#ifndef __DM_IMU_H__
#define __DM_IMU_H__
#include "component/common/types.h"
typedef struct
{
    uint8_t  id;
    float accel[3];//分别对应xyz
    float angular[3];
    float euler[3];
    float quaternion[4];//
} dm_imu_frame_t;


typedef enum 
{
    IMU_STATE_ACCEL,
    IMU_STATE_ANGULAR,
    IMU_STATE_EULER,
    IMU_STATE_QUATERNION,
}DM_IMU_NEXT_CORRESPONDING_STATE_E;



#endif