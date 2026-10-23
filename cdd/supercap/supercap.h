#ifndef CDD_SUPERCAP_H__
#define CDD_SUPERCAP_H__
#include "component/common/types.h"

/* 超级电容，全车只有一个。CAN 标识符固定：状态帧 0x05A，控制帧 0x06A。挂哪条总线见 supercap_config.h。
 *
 * 状态帧 0x05A：
 *   byte0     错误等级，只用低两位：0 无错，1 可自恢复，2 发命令可恢复，3 不可恢复
 *   byte1..4  底盘功率 float，W
 *   byte5..6  底盘功率上限 uint16 小端，W
 *   byte7     电容能量 0..255，满量程按 2000 J
 * 控制帧 0x06A：
 *   byte0     bit0 使能 DCDC，bit1 重启，bit2 清错误
 *   byte1..2  裁判功率上限 uint16 小端，W
 *   byte3..4  裁判缓冲能量 uint16 小端，J */

#define SUPERCAP_ENERGY_FULL_J  2000.0f

typedef enum
{
    SUPERCAP_ERROR_NONE = 0,
    SUPERCAP_ERROR_AUTO = 1,
    SUPERCAP_ERROR_CMD  = 2,
    SUPERCAP_ERROR_FATAL = 3,
} supercap_error_e;

typedef struct
{
    bool             online;
    supercap_error_e error;
    float            chassis_power_w;
    uint16_t         chassis_power_limit_w;
    uint8_t          energy_raw;          /* 0..255 */
    float            energy_j;
} supercap_status_t;

void supercap_enable(bool on);
/* 下一帧控制帧里带上裁判功率上限和缓冲能量。功率限制在 35..250 W */
void supercap_set_referee(uint16_t power_limit_w, uint16_t buffer_j);
void supercap_get(supercap_status_t *out);
/* 发一帧 0x06A。错误等级为 2 时会按间隔置位清错误，多次仍不恢复再置位重启 */
int32_t supercap_send(void);
#endif
