#ifndef ROBOT_CFG_H__
#define ROBOT_CFG_H__
/* demo 兵种的构型参数：哪条总线挂哪些电机。
 * 模块要不要编译、功能开不开，不在这里，在 robot/demo/demo_h7.config 的 CONFIG_XXX = y/n。 */

/* ---- 底盘电机绑定（CONFIG_ROBOT_BIND_CHASSIS = y 时生效）----
 * 4 个 M3508 挂在哪条 CAN、ID 是多少。打开前确认总线上真有电机：
 * 经典 CAN 开了自动重传，发给无应答总线的帧会被不断重发，最终总线错误。 */
#include "bsp/include/bsp_ids.h"

#define CHASSIS_BUS             CAN_BUS0
#define CHASSIS_ID_BASE         1            /* FL FR BL BR = id 1..4 */

#endif
