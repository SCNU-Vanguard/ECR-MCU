#ifndef HOST_CONFIG_H__
#define HOST_CONFIG_H__
#include "component/common/types.h"
#include "osal/osal.h"
#include "cdd/host/host.h"

/* 要哪些命令就写进 HOST_PACKET_TABLE。载荷 1 字节 packed，上限 255。 */
#define HOST_PAYLOAD_MAX  32u
/* 板向上位机发送的频率，填 OS_FREQ_xxHZ。 */
#define HOST_TX_HZ  OS_FREQ_100HZ

_Static_assert(HOST_PAYLOAD_MAX <= 255u, "host payload length field is 1 byte");

#define HOST_CMD_AIM_ATTITUDE  0x01u
#define HOST_CMD_AIM_TARGET    0x02u
#define HOST_CMD_NAV_STATE     0x03u
#define HOST_CMD_NAV_SPEED     0x04u

typedef struct BYTE_PACKED
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} host_aim_attitude_t;

typedef struct BYTE_PACKED
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    uint8_t is_locked;
    uint8_t shoot_en;
} host_aim_target_t;

typedef struct BYTE_PACKED
{
    uint16_t hp;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} host_nav_state_t;

typedef struct BYTE_PACKED
{
    float forward_m_s;
    float right_m_s;
    float spin_rad_s;
} host_nav_speed_t;

_Static_assert(sizeof(host_aim_attitude_t) <= HOST_PAYLOAD_MAX, "host_aim_attitude_t exceeds HOST_PAYLOAD_MAX");
_Static_assert(sizeof(host_aim_target_t) <= HOST_PAYLOAD_MAX, "host_aim_target_t exceeds HOST_PAYLOAD_MAX");
_Static_assert(sizeof(host_nav_state_t) <= HOST_PAYLOAD_MAX, "host_nav_state_t exceeds HOST_PAYLOAD_MAX");
_Static_assert(sizeof(host_nav_speed_t) <= HOST_PAYLOAD_MAX, "host_nav_speed_t exceeds HOST_PAYLOAD_MAX");

#define HOST_PACKET_TABLE(X)                                                                                    \
    X(HOST_CMD_AIM_ATTITUDE, HOST_SEND, host_aim_attitude_t) /* 自瞄姿态 */                                      \
    X(HOST_CMD_AIM_TARGET,   HOST_RECV, host_aim_target_t)   /* 自瞄目标。pitch、yaw 全为 0 时云台用手动 */       \
    X(HOST_CMD_NAV_STATE,    HOST_SEND, host_nav_state_t)    /* 自身血量 + 姿态。血量没有来源时发 0 */            \
    X(HOST_CMD_NAV_SPEED,    HOST_RECV, host_nav_speed_t)    /* 目标移速。三个分量全为 0 时底盘仍用手动 */
/*    命令号 ，              方向 ，     载荷 */
#endif
