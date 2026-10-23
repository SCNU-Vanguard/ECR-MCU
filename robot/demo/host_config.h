#ifndef HOST_CONFIG_H__
#define HOST_CONFIG_H__
#include "component/common/types.h"
#include "cdd/host/host.h"

/* 上位机命令。外层帧在 cdd/host，载荷按 1 字节 packed。
 * 自瞄由 CONFIG_CDD_HOST_AIM 打开，导航由 CONFIG_CDD_HOST_NAV 打开。两个都可以关。
 * 每一项带方向：HOST_SEND 是板发出去的，HOST_RECV 是上位机发来的。
 * 长度字段是 1 字节，所以上限不能超过 255。
 * HOST_PACKET_TABLE 在 cdd/host 里展开成静态缓存并注册。 */

#define HOST_PAYLOAD_MAX  32u

#ifdef CONFIG_CDD_HOST_AIM
#define HOST_CMD_AIM_ATTITUDE  0x01u   /* 板 → 上位机：自瞄姿态 */
#define HOST_CMD_AIM_TARGET    0x02u   /* 上位机 → 板：自瞄目标 */

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

#define HOST_AIM_PACKETS(X)                                                    \
    X(AIM_ATTITUDE, HOST_CMD_AIM_ATTITUDE, HOST_SEND, host_aim_attitude_t)     \
    X(AIM_TARGET,   HOST_CMD_AIM_TARGET,   HOST_RECV, host_aim_target_t)
#else
#define HOST_AIM_PACKETS(X)
#endif

#ifdef CONFIG_CDD_HOST_NAV
#define HOST_CMD_NAV_STATE     0x03u   /* 板 → 上位机：自身血量 + 姿态 */
#define HOST_CMD_NAV_SPEED     0x04u   /* 上位机 → 板：目标移速 */

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

#define HOST_NAV_PACKETS(X)                                                    \
    X(NAV_STATE, HOST_CMD_NAV_STATE, HOST_SEND, host_nav_state_t)              \
    X(NAV_SPEED, HOST_CMD_NAV_SPEED, HOST_RECV, host_nav_speed_t)
#else
#define HOST_NAV_PACKETS(X)
#endif

#define HOST_PACKET_TABLE(X)                                                   \
    HOST_AIM_PACKETS(X)                                                        \
    HOST_NAV_PACKETS(X)

#endif
