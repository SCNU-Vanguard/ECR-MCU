#ifndef CDD_LK_MOTOR_H__
#define CDD_LK_MOTOR_H__
#include "component/common/types.h"
#include "bsp/include/bsp_ids.h"
#include "cdd/motor/motor.h"

/* 瓴控 MF9025。协议见《电机 CAN 总线通讯协议》V2.36，英雄底盘云台用的就是这一台。
 * 标识符 = 0x140 + id（id 1..32），命令和回复用同一个 ID。
 * 控制用转矩闭环 0xA1：iq 为 int16，±2048 对应 MF 系列 ±16.5 A。
 * 回复布局和“读取状态 2”（0x9C）相同：温度、iq、转速 dps、16 位编码器。
 * 反馈是问询式的，每个控制周期都要发一帧才会有回复。 */

typedef struct
{
    can_id_e     bus;
    uint8_t      id;                 /* 1..32，线上 ID = 0x140 + id */
    bool         reverse;
    uint32_t     timeout_ms;
    const motor_control_config_t *control;   /* NULL = 只用 lk_motor_set_iq 给原始 iq */
} lk_motor_config_t;

typedef struct
{
    motor_t  base;
    uint8_t  id;
    int16_t  iq;                     /* 由 output 换算出的待发送 iq，已限幅、已处理 reverse */
    uint16_t encoder;                /* 0..65535 */
    uint16_t last_encoder;
    int32_t  round_count;
    int16_t  speed_dps;
    int16_t  current;                /* 反馈 iq，未应用 reverse */
} lk_motor_t;

int32_t lk_motor_register(lk_motor_t *motor, const lk_motor_config_t *config);
/* 直接给 iq 原始值，绕过软件控制。仅对 control == NULL 的电机生效 */
void    lk_motor_set_iq(lk_motor_t *motor, int16_t iq);
/* 0x88 运行、0x81 停止（不清圈数）、0x9B 清错误。0x80 关闭会清圈数，这里不提供 */
int32_t lk_motor_run(lk_motor_t *motor);
int32_t lk_motor_stop(lk_motor_t *motor);
int32_t lk_motor_clear_error(lk_motor_t *motor);

void    lk_motor_compute(lk_motor_t *motor);
int32_t lk_motor_compute_bus(can_id_e bus);
int32_t lk_motor_send(lk_motor_t *motor);
int32_t lk_motor_send_bus(can_id_e bus);
void    lk_motor_zero(lk_motor_t *motor);
#endif
