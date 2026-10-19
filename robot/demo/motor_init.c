#include "robot/demo/motor_config.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/log/log.h"

/* 电机绑定：按 motor_config.h 里的表，为每个电机定义变量并绑定到 CAN 总线。
 * 这里只做注册和绑定，不关心电机属于哪个功能模块。 */

#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
/* 控制配置由 motor_config.h 的控制表展开。模式在表里写死。 */
#define ROBOT_DJI_MOTOR_CONTROL(_name, _mode, _akp, _aki, _akd, _akf, _aout, _aint, _aff, _adb, \
                          _skp, _ski, _skd, _skf, _sout, _sint, _sff, _sdb) \
    static const motor_control_config_t _name##_control =                                                  \
    {                                                                                                      \
        .mode = (_mode),                                                                                   \
        .angle_pid = { _akp, _aki, _akd, _akf, _aout, _aint, _aff, _adb },                                 \
        .speed_pid = { _skp, _ski, _skd, _skf, _sout, _sint, _sff, _sdb },                                 \
    };
ROBOT_DJI_MOTOR_CONTROL_TABLE(ROBOT_DJI_MOTOR_CONTROL)
#undef ROBOT_DJI_MOTOR_CONTROL
#define ROBOT_DEFINE_DJI_MOTOR(_name, ...)    dji_motor_t _name;
ROBOT_DJI_MOTOR_CONFIG_TABLE(ROBOT_DEFINE_DJI_MOTOR)

__attribute__((unused))
static int32_t bind_dji_motor(const char *name, dji_motor_t *motor, const dji_motor_config_t *config)
{
    int32_t register_ret = dji_motor_register(motor, config);

    if (register_ret != ERR_OK)
    {
        LOGE("bind dji motor %s failed: %d", name, (int)register_ret);
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
#define ROBOT_DM_MOTOR_CONTROL(_name, _mode, _akp, _aki, _akd, _akf, _aout, _aint, _aff, _adb, \
                         _skp, _ski, _skd, _skf, _sout, _sint, _sff, _sdb) \
    static const motor_control_config_t _name##_control =                                                 \
    {                                                                                                     \
        .mode = (_mode),                                                                                  \
        .angle_pid = { _akp, _aki, _akd, _akf, _aout, _aint, _aff, _adb },                                \
        .speed_pid = { _skp, _ski, _skd, _skf, _sout, _sint, _sff, _sdb },                                \
    };
ROBOT_DM_MOTOR_CONTROL_TABLE(ROBOT_DM_MOTOR_CONTROL)
#undef ROBOT_DM_MOTOR_CONTROL
#define ROBOT_DEFINE_DM_MOTOR(_name, ...)     dm_motor_t _name;
ROBOT_DM_MOTOR_CONFIG_TABLE(ROBOT_DEFINE_DM_MOTOR)

__attribute__((unused))
static int32_t bind_dm_motor(const char *name, dm_motor_t *motor, const dm_motor_config_t *config)
{
    int32_t register_ret = dm_motor_register(motor, config);

    if (register_ret != ERR_OK)
    {
        LOGE("bind dm motor %s failed: %d", name, (int)register_ret);
    }
    return register_ret;
}

#define ROBOT_BIND_DM_MOTOR(_name, _bus, _type, _tx_id, _master_id, _mode, _reverse, _p_max, _v_max, _t_max, _timeout_ms, _kp, _kd, _speed_limit, _current_limit, _control)    \
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

static int32_t motor_bind_init(void)
{
    int32_t register_ret = ERR_OK;

#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
    ROBOT_DJI_MOTOR_CONFIG_TABLE(ROBOT_BIND_DJI_MOTOR)
#endif
#ifdef CONFIG_CDD_MOTOR_DM_MOTOR
    ROBOT_DM_MOTOR_CONFIG_TABLE(ROBOT_BIND_DM_MOTOR)
#endif
    return register_ret;
}
INIT_APP_STAGE1(motor_bind_init);
