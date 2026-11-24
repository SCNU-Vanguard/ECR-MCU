#ifndef CHASSIS_CONFIG_H__
#define CHASSIS_CONFIG_H__
#include "component/chassis/chassis.h"

/* 占位，数值与 demo 相同。本车任务接上之前按实车改几何、减速比和功率模型。
 * 构型要和 robot.config 里打开的解算一致。 */
#define CHASSIS_TYPE                       CHASSIS_TYPE_OMNI
#define CHASSIS_HALF_LENGTH_M              0.30f
#define CHASSIS_HALF_WIDTH_M               0.30f
#define CHASSIS_WHEEL_RADIUS_M             0.10f
#define CHASSIS_DRIVE_RATIO                19.0f
#define CHASSIS_POWER_K_OUTPUT_SPEED       1.9069e-5f
#define CHASSIS_POWER_K_SPEED2             1.8535e-5f
#define CHASSIS_POWER_K_OUTPUT2            1.453e-7f
#define CHASSIS_POWER_IDLE_W               0.75f
#define CHASSIS_POWER_EXTRA_W              25.0f
#define CHASSIS_POWER_MARGIN_W             10.0f
#define CHASSIS_POWER_ENERGY_HIGH_RAW      38u
#endif
