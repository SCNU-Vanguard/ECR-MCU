#ifndef SUPERCAP_CONFIG_H__
#define SUPERCAP_CONFIG_H__
/* 一台机器只有一个超级电容。收 0x05A、发 0x06A，和英雄、步兵、标准相同。
 * 它们都挂在底盘 CAN（fdcan2）上，这里是 CAN_BUS1。 */
#include "bsp/include/bsp_ids.h"

#define SUPERCAP_BUS          CAN_BUS1
#define SUPERCAP_TIMEOUT_MS   100u
#define SUPERCAP_POWER_W      45u    /* 还没收到裁判功率上限时用这个 */
#define SUPERCAP_BUFFER_J     60u
#endif
