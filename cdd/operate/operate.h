#ifndef CDD_OPERATE_H__
#define CDD_OPERATE_H__
#include "component/common/types.h"
#include "component/common/err.h"

/* 操作手。DT7、图传遥控、图传键盘收成同一组寄存器，用哪种由兵种 operate_config.h 决定。
 * 图传遥控和蓝牙遥控是同一帧：蓝牙模块接在图传串口上，imgtrans 只收不发。
 * 这里只记录操作，不因使能、导航、摩擦轮清掉别的寄存器。
 * 导航速度和自瞄目标由应用层自己取。 */

#define OPERATE_MODE_DT7            0
#define OPERATE_MODE_IMGTRANS_RC    1
#define OPERATE_MODE_IMGTRANS_KEY   2

/* 低 16 位和 dt7 / imgtrans 的键盘位图一致。鼠标键放在高位，供改键使用。 */
#define OPERATE_KEY_W         (1u << 0)
#define OPERATE_KEY_S         (1u << 1)
#define OPERATE_KEY_A         (1u << 2)
#define OPERATE_KEY_D         (1u << 3)
#define OPERATE_KEY_SHIFT     (1u << 4)
#define OPERATE_KEY_CTRL      (1u << 5)
#define OPERATE_KEY_Q         (1u << 6)
#define OPERATE_KEY_E         (1u << 7)
#define OPERATE_KEY_R         (1u << 8)
#define OPERATE_KEY_F         (1u << 9)
#define OPERATE_KEY_G         (1u << 10)
#define OPERATE_KEY_Z         (1u << 11)
#define OPERATE_KEY_X         (1u << 12)
#define OPERATE_KEY_C         (1u << 13)
#define OPERATE_KEY_V         (1u << 14)
#define OPERATE_KEY_B         (1u << 15)
#define OPERATE_MOUSE_LEFT    (1u << 16)
#define OPERATE_MOUSE_RIGHT   (1u << 17)
#define OPERATE_MOUSE_MIDDLE  (1u << 18)

/* 注册项的处理方式，只作用在键上。按住：输出跟着键。切换：键的上升沿翻转，掉线清掉。 */
#define OPERATE_AS_HOLD     0u
#define OPERATE_AS_TOGGLE   1u

/* 键盘映射表的输入槽，只给 operate_config.h 的 OPERATE_REG_TABLE 用，与 operate_snap_t 的字段无关。
 * 图传遥控（OPERATE_MODE_IMGTRANS_RC）不查表，遥控量到快照字段的对应写死在 operate.c。
 * 新增一个键控开关：加一个槽，表里加一行，operate.c 里把槽赋给快照字段。 */
enum
{
    OPERATE_IN_ENABLE = 0,
    OPERATE_IN_FRICTION,
    OPERATE_IN_NAV,
    OPERATE_IN_AIM,
    OPERATE_IN_SHOOT,
    OPERATE_IN_SPIN,
    OPERATE_IN_FORWARD,
    OPERATE_IN_BACK,
    OPERATE_IN_LEFT,
    OPERATE_IN_RIGHT,
    OPERATE_IN_CUSTOM1,
    OPERATE_IN_CUSTOM2,
    OPERATE_IN_CUSTOM3,
    OPERATE_IN_CUSTOM4,
    OPERATE_IN_COUNT
};

/* 一次发布的整份快照，唯一的对外接口，改字段只改这里和 operate.c。
 * forward / right 是 m/s，spin、yaw_rate、pitch_rate 是 rad/s，带正负。
 * yaw_rate 向右为负，pitch_rate 向上为正。摇杆杆量和鼠标计数乘灵敏度都写进这两个转速。
 * 应用层按自己的任务周期把转速加到目标角上。 */
typedef struct
{
    bool     online;
    bool     enabled;
    float    forward_m_s;
    float    right_m_s;
    float    spin_rad_s;
    bool     use_nav;
    bool     use_aim;
    float    yaw_rate_rad_s;
    float    pitch_rate_rad_s;
    bool     friction;
    bool     shoot;
    bool     custom1;
    bool     custom2;
    bool     custom3;
    bool     custom4;
} operate_snap_t;

/* 一次拷出整份快照。控制周期里应取一次再用各字段。out 为空时返回 ERR_INVAL。 */
int32_t operate_get_snapshot(operate_snap_t *out);
#endif
