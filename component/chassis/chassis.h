#ifndef COMPONENT_CHASSIS_H__
#define COMPONENT_CHASSIS_H__
#include "cdd/motor/motor.h"

/* 解算用的底盘坐标系：x 向前，y 向右，自旋俯视逆时针为正。三个速度都是底盘中心的速度。
 * 数组下标 0–3 只按数学象限编号，不参与公式。俯视、车头朝上时，数学坐标的 +x 向右、+y 向前：
 *   1 左前（第二象限）    0 右前（第一象限）
 *   2 左后（第三象限）    3 右后（第四象限）
 * 麦轮正角速度是把底盘向前推的方向。全向轮正角速度沿该轮 45° 的驱动方向，纯前进时右侧两轮为负。
 * 电机装反时用电机自己的 reverse，不写进解算。
 *
 * 构型、几何、驱动减速比和功率模型在当前兵种的 chassis_config.h。
 * chassis_solve / chassis_inverse 按 CHASSIS_TYPE 选解算，声明也按构型分开。
 * 应用层只传入电机：驱动电机下标和上面的轮序一致。减速比在模块内乘除。
 * 全向轮和麦轮只有驱动电机。舵轮再传入同轮序的转向电机。 */

#define CHASSIS_TYPE_OMNI       0
#define CHASSIS_TYPE_MECANUM    1
#define CHASSIS_TYPE_SWERVE     2

enum
{
    CHASSIS_WHEEL_FRONT_RIGHT = 0,    /* 第一象限：右前 */
    CHASSIS_WHEEL_FRONT_LEFT,         /* 第二象限：左前 */
    CHASSIS_WHEEL_REAR_LEFT,          /* 第三象限：左后 */
    CHASSIS_WHEEL_REAR_RIGHT,         /* 第四象限：右后 */
    CHASSIS_WHEEL_COUNT
};

typedef struct
{
    float forward_m_s; /* 向前为正 */
    float right_m_s;   /* 向右为正 */
    float spin_rad_s;  /* 俯视逆时针为正 */
} chassis_speed_t;

typedef struct
{
    float half_length_m;    /* 几何中心到前/后轮轴 */
    float half_width_m;     /* 几何中心到左/右轮 */
    float wheel_radius_m;
} chassis_geometry_t;

#include "chassis_config.h"

/* 正解：底盘中心速度写成驱动电机目标。几何或减速比非法、驱动电机有空指针：返回 ERR_INVAL，不写目标。
 * 逆解：读驱动电机转速，得到底盘中心速度。空指针或几何、减速比非法：返回 ERR_INVAL。
 * 有电机离线：返回 ERR_FAIL，不写 speed。选了没打开的构型时编译失败。
 * 舵轮同时读写转向电机。转向电机角度 = 轮端舵角 × CHASSIS_STEER_RATIO；配置里没写时按 1。
 * 缺转向电机时正解和逆解都返回 ERR_INVAL。 */
#if CHASSIS_TYPE == CHASSIS_TYPE_OMNI || CHASSIS_TYPE == CHASSIS_TYPE_MECANUM
int32_t chassis_solve(motor_t *const drive[CHASSIS_WHEEL_COUNT], const chassis_speed_t *speed);
int32_t chassis_inverse(motor_t *const drive[CHASSIS_WHEEL_COUNT], chassis_speed_t *speed);
#elif CHASSIS_TYPE == CHASSIS_TYPE_SWERVE
int32_t chassis_solve(motor_t *const drive[CHASSIS_WHEEL_COUNT],
                      motor_t *const steer[CHASSIS_WHEEL_COUNT],
                      const chassis_speed_t *speed);
int32_t chassis_inverse(motor_t *const drive[CHASSIS_WHEEL_COUNT],
                        motor_t *const steer[CHASSIS_WHEEL_COUNT],
                        chassis_speed_t *speed);
#else
#error "CHASSIS_TYPE must be CHASSIS_TYPE_OMNI, CHASSIS_TYPE_MECANUM, or CHASSIS_TYPE_SWERVE"
#endif

/* 裁判上限和超电能量。加多少、减多少由 chassis_config.h 决定。任一来源无效就当没有。 */
typedef struct
{
    bool     referee_valid;
    float    referee_limit_w;
    bool     supercap_valid;       /* 在线且没有不可恢复错误 */
    uint8_t  energy_raw;           /* 0..255 */
    float    supercap_limit_w;
} chassis_power_source_t;

/* 返回值 <= 0：不限制。source 为空，或两个来源都无效：返回 0。 */
float chassis_power_budget_w(const chassis_power_source_t *source);

/* 用本车驱动电机功率模型限制这些电机的 output。只改传入的电机，转向和关节不要放进来。
 * max_power_w <= 0、motor 为空、count 为 0 或大于 16：不改 output，返回 0。
 * 返回削减前的预测正功率。 */
float chassis_power_apply(motor_t *const motor[], uint8_t count, float max_power_w);
#endif
