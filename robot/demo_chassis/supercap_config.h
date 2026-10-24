#ifndef SUPERCAP_CONFIG_H__
#define SUPERCAP_CONFIG_H__
/* 收 0x05A、发 0x06A。和 yaw 的 j4310 同在 FDCAN2。 */
#include "bsp/include/bsp_ids.h"

#define SUPERCAP_BUS          CAN_BUS2
#define SUPERCAP_TIMEOUT_MS   100u
#define SUPERCAP_POWER_W      45u    /* 还没收到裁判功率上限时用这个 */
#define SUPERCAP_BUFFER_J     60u
#endif
