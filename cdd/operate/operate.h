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

/* 图传遥控读哪一路。键盘模式不看。这一路是遥控器上拨好的电平，按住跟着走。 */
#define OPERATE_RC_NONE         0u
#define OPERATE_RC_PAUSE        1u
#define OPERATE_RC_CUSTOM_LEFT  2u
#define OPERATE_RC_CUSTOM_RIGHT 3u
#define OPERATE_RC_TRIGGER      4u
#define OPERATE_RC_AIM          5u
#define OPERATE_RC_NAV          6u
#define OPERATE_RC_SPIN         7u

/* 查询寄存器。operate_get 的 size 必须等于类型宽度。
 * FORWARD / RIGHT 是 m/s，SPIN 是 rad/s，带正负。
 * YAW_DELTA 向右为负，PITCH_DELTA 向上为正，是这一帧要转的角。
 * 图传遥控和 DT7 只取摇杆通道，角速度乘上一帧时间戳到这一帧的时间。图传键盘只加鼠标。
 * SEQ 每来一帧输入加一。seq 变了再把这份增量加上去。 */
enum
{
    OPERATE_ID_ONLINE = 0,      /* bool */
    OPERATE_ID_ENABLE,          /* bool */
    OPERATE_ID_FORWARD,         /* float */
    OPERATE_ID_RIGHT,           /* float */
    OPERATE_ID_SPIN,            /* float */
    OPERATE_ID_NAV,             /* bool */
    OPERATE_ID_AIM,             /* bool */
    OPERATE_ID_YAW_DELTA,       /* float */
    OPERATE_ID_PITCH_DELTA,     /* float */
    OPERATE_ID_SEQ,             /* uint32_t */
    OPERATE_ID_FRICTION,        /* bool */
    OPERATE_ID_SHOOT,           /* bool */
    OPERATE_ID_CUSTOM,          /* bool */
    OPERATE_ID_COUNT
};

/* 注册表的输入槽，只给 operate_config.h 的 OPERATE_REG_TABLE 用。 */
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
    OPERATE_IN_CUSTOM,
    OPERATE_IN_COUNT
};

/* 拷出这一项。编号不存在或 size 与该项宽度不符时返回 ERR_INVAL，不写 buf。 */
int32_t operate_get(unsigned id, void *buf, size_t size);
#endif
