#ifndef CDD_MOTOR_INIT_H__
#define CDD_MOTOR_INIT_H__
/* 当前兵种的电机对象。定义在 motor_init.c，名字来自 motor_config.h 的配置表。
 * 应用层要引用某台电机时包含这个头，不要在各任务里再写一份 extern。 */
#include "motor_config.h"

#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
#include "cdd/motor/dji_motor/dji_motor.h"
#define ROBOT_DECLARE_DJI_MOTOR(_name, ...)    extern dji_motor_t _name;
ROBOT_DJI_MOTOR_CONFIG_TABLE(ROBOT_DECLARE_DJI_MOTOR)
#undef ROBOT_DECLARE_DJI_MOTOR
#endif

#ifdef CONFIG_CDD_MOTOR_DM_MOTOR
#include "cdd/motor/dm_motor/dm_motor.h"
#define ROBOT_DECLARE_DM_MOTOR(_name, ...)     extern dm_motor_t _name;
ROBOT_DM_MOTOR_CONFIG_TABLE(ROBOT_DECLARE_DM_MOTOR)
#undef ROBOT_DECLARE_DM_MOTOR
#endif

#endif
