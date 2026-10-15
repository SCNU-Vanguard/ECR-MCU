#include "robot/demo/robot_cfg.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/log/log.h"

/* 电机绑定：按 robot_cfg.h 里的表，为每个电机定义变量并绑定到 CAN 总线。
 * 这里只做注册和绑定，不关心电机属于哪个功能模块。 */

#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
/* 各电机的软件控制配置（PID 参数）。这些是没有实测过的保守初值：电机会动得很弱甚至不动，现场用 shell 的 xxx_pid 命令调。
 * 单位：角度环 rad -> rad/s；速度环 rad/s -> 电流/电压原始值（DJI）。
 * 列含义：
 *   kp | ki | kd | kf | 输出限幅 | 积分限幅 | 前馈限幅 | 死区
 * 角度环是位置式（ki 累加进积分，积分限幅限的是累加值）；速度环是增量式（积分限幅限的是 ki*误差 这一项）。 */
static const motor_control_config_t shoot_motor_control =
{
    .angle_pid = { 0.0f,  0.0f, 0.0f, 0.0f,    0.0f,    0.0f, 0.0f, 0.0f },
    .speed_pid = { 10.0f, 1.0f, 0.0f, 0.0f, 2000.0f,  500.0f, 0.0f, 0.0f },
};

static const motor_control_config_t chassis_motor_control =
{
    .angle_pid = {  5.0f, 0.0f, 0.0f, 0.0f,   10.0f,    0.0f, 0.0f, 0.0f },
    .speed_pid = { 30.0f, 3.0f, 0.0f, 0.0f, 4000.0f, 1000.0f, 0.0f, 0.0f },
};
#define ROBOT_DEFINE_DJI_MOTOR(_name, ...)    dji_motor_t _name;
ROBOT_DJI_MOTOR_TABLE(ROBOT_DEFINE_DJI_MOTOR)

__attribute__((unused))
static int32_t bind_dji_motor(const char *name, dji_motor_t *motor, const dji_motor_config_t *config)
{
    int32_t register_ret = dji_motor_register(motor, config);

    if (register_ret != ERR_OK)
    {
        LOGE("robot", "bind dji motor %s failed: %d", name, (int)register_ret);
    }
    return register_ret;
}

#define ROBOT_BIND_DJI_MOTOR(_name, _bus, _type, _id, _reverse, _offset_encoder, _timeout_ms, _gm6020_mode, _control)    \
    {                                                                                            \
        const dji_motor_config_t config =                                                        \
        {                                                                                        \
            .bus            = (_bus),                                                            \
            .type           = (_type),                                                           \
            .id             = (_id),                                                             \
            .reverse        = (_reverse),                                                        \
            .offset_encoder = (_offset_encoder),                                                 \
            .timeout_ms     = (_timeout_ms),                                                     \
            .gm6020_mode    = (_gm6020_mode),                                                    \
            .control        = (_control),                                                        \
        };                                                                                       \
                                                                                                 \
        register_ret = bind_dji_motor(#_name, &_name, &config);                                  \
        if (register_ret != ERR_OK)                                                              \
        {                                                                                        \
            return register_ret;                                                                 \
        }                                                                                        \
    }
#endif

#ifdef CONFIG_CDD_MOTOR_DM_MOTOR
/* DM 电机的软件控制配置。MIT 软件转矩：角度环 rad -> rad/s，速度环 rad/s -> 转矩 N·m。
 * 列含义同上；没有实测过，输出限幅 1 N·m（DM4310 的 t_max 是 10 N·m）。 */
static const motor_control_config_t gimbal_motor_control =
{
    .angle_pid = { 5.0f, 0.0f,  0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f },
    .speed_pid = { 0.2f, 0.02f, 0.0f, 0.0f, 1.0f, 0.2f, 0.0f, 0.0f },
};
#define ROBOT_DEFINE_DM_MOTOR(_name, ...)     dm_motor_t _name;
ROBOT_DM_MOTOR_TABLE(ROBOT_DEFINE_DM_MOTOR)

__attribute__((unused))
static int32_t bind_dm_motor(const char *name, dm_motor_t *motor, const dm_motor_config_t *config)
{
    int32_t register_ret = dm_motor_register(motor, config);

    if (register_ret != ERR_OK)
    {
        LOGE("robot", "bind dm motor %s failed: %d", name, (int)register_ret);
    }
    return register_ret;
}

#define ROBOT_BIND_DM_MOTOR(_name, _bus, _type, _tx_id, _master_id, _mode, _reverse, _p_max, _v_max, _t_max, _timeout_ms, _mit_style, _kp, _kd, _speed_limit, _current_limit, _control)    \
    {                                                                                            \
        const dm_motor_config_t config =                                                         \
        {                                                                                        \
            .bus        = (_bus),                                                                \
            .type       = (_type),                                                               \
            .tx_id      = (_tx_id),                                                              \
            .master_id  = (_master_id),                                                          \
            .mode       = (_mode),                                                               \
            .reverse    = (_reverse),                                                            \
            .p_max      = (_p_max),                                                              \
            .v_max      = (_v_max),                                                              \
            .t_max      = (_t_max),                                                              \
            .timeout_ms = (_timeout_ms),                                                         \
            .mit_style  = (_mit_style),                                                          \
            .kp         = (_kp),                                                                 \
            .kd         = (_kd),                                                                 \
            .speed_limit   = (_speed_limit),                                                     \
            .current_limit = (_current_limit),                                                   \
            .control    = (_control),                                                            \
        };                                                                                       \
                                                                                                 \
        register_ret = bind_dm_motor(#_name, &_name, &config);                                   \
        if (register_ret != ERR_OK)                                                              \
        {                                                                                        \
            return register_ret;                                                                 \
        }                                                                                        \
    }
#endif

static int32_t robot_bind_motors(void)
{
    int32_t register_ret = ERR_OK;

#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
    ROBOT_DJI_MOTOR_TABLE(ROBOT_BIND_DJI_MOTOR)
#endif
#ifdef CONFIG_CDD_MOTOR_DM_MOTOR
    ROBOT_DM_MOTOR_TABLE(ROBOT_BIND_DM_MOTOR)
#endif
    return register_ret;
}
INIT_APP_STAGE1(robot_bind_motors);
