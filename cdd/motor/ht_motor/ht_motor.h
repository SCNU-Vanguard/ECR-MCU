#ifndef CDD_HIGH_TORQUE_MOTOR_H__
#define CDD_HIGH_TORQUE_MOTOR_H__
#include "component/common/types.h"
#include "bsp/include/bsp_ids.h"
#include "cdd/motor/motor.h"



#endif
#ifndef CDD_HT_MOTOR_H__
#define CDD_HT_MOTOR_H__

#include "cdd/motor/motor.h"

typedef enum
{
    HT_MOTOR_MODE_TORQUE,
    HT_MOTOR_MODE_POSITION_SPEED,
    HT_MOTOR_MODE_MIT,
} ht_motor_mode_e;

typedef struct
{
    can_id_e bus;
    motor_type_e type;
    uint8_t id;           /* 电机地址为 1..126，主机地址为 0 */
    ht_motor_mode_e mode;
    bool reverse;
    uint32_t timeout_ms;
    float torque_scale;   /* 每个协议力矩单位对应的实测力矩（N·m），必须大于 0 */
    float torque_limit;   /* 力矩给定和位置速度模式的力矩上限，N·m */
    float speed_limit;    /* 位置速度模式的速度上限，rad/s */
    float kp;             /* MIT 模式的位置刚度，N·m/rad */
    float kd;             /* MIT 模式的速度阻尼，N·m/(rad/s) */
    const motor_control_config_t *control;
} ht_motor_config_t;

typedef struct
{
    motor_t base;
    ht_motor_config_t config;
    float cmd_position;   /* 电机方向的位置给定，rad */
    float cmd_velocity;   /* 电机方向的速度给定，rad/s */
    float cmd_kp;
    float cmd_kd;
    bool cmd_valid;
    bool braking;
    uint8_t state;
    uint8_t fault;
} ht_motor_t;

/* 在 INIT_APP_STAGE1 注册，注册阶段只绑定电机和订阅反馈。 */
int32_t ht_motor_register(ht_motor_t *motor, const ht_motor_config_t *config);
void ht_motor_compute(ht_motor_t *motor);
int32_t ht_motor_compute_bus(can_id_e bus);
int32_t ht_motor_send(ht_motor_t *motor);
int32_t ht_motor_send_bus(can_id_e bus);
int32_t ht_motor_disable(ht_motor_t *motor);
int32_t ht_motor_brake(ht_motor_t *motor);
int32_t ht_motor_read_state(ht_motor_t *motor);

#endif
