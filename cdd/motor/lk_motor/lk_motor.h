#ifndef CDD_LK_MOTOR_H__
#define CDD_LK_MOTOR_H__
#include "component/common/types.h"
#include "bsp/include/bsp_ids.h"
#include "cdd/motor/motor.h"

/* lk_motor（瓴控 mf9025）的硬件控制模式。电机按收到的命令字节决定闭环方式，上位机里不用切模式。
 *   TORQUE：0xA1 转矩闭环。软件做速度环/角度环，output 为 iq 原始值 ±2048（mf 对应 ±16.5 A）。
 *           发送前应用层可以改 output；没有控制配置时用 lk_motor_set_cmd 直接给原始值
 *   SPEED： 0xA2 速度闭环，电机内部速度环，iq_limit 为转矩电流上限。
 *           SPEED 控制模式：target（rad/s）直接作为速度给定，不需要控制配置；
 *           ANGLE 控制模式：软件角度环输出作为速度给定，需要控制配置
 * 两种模式下 active = false、还没收到反馈、或不支持的组合都发 0xA1 iq = 0：不出力，照样回复反馈 */
typedef enum
{
    LK_MOTOR_MODE_TORQUE,
    LK_MOTOR_MODE_SPEED,
} lk_motor_mode_e;

/* 错误标志：状态 1 回复里的 errorState，非 0 时 base.error_flags 置 MOTOR_ERROR_FAULT */
#define LK_MOTOR_ERROR_UNDER_VOLTAGE   (1u << 0)
#define LK_MOTOR_ERROR_OVER_VOLTAGE    (1u << 1)
#define LK_MOTOR_ERROR_DRIVER_TEMP     (1u << 2)
#define LK_MOTOR_ERROR_MOTOR_TEMP      (1u << 3)
#define LK_MOTOR_ERROR_OVER_CURRENT    (1u << 4)
#define LK_MOTOR_ERROR_SHORT_CIRCUIT   (1u << 5)
#define LK_MOTOR_ERROR_STALL           (1u << 6)
#define LK_MOTOR_ERROR_INPUT_TIMEOUT   (1u << 7)
/* 电机状态：状态 1 回复里的 motorState */
#define LK_MOTOR_STATE_ON              0x00u
#define LK_MOTOR_STATE_OFF             0x10u

/* 故障标志只在状态 1 里：每成功发出这么多帧控制帧，额外问询一次（1 kHz 控制周期下为 10 Hz） */
#define LK_MOTOR_STATE1_POLL_PERIOD    100u

/* 注册配置：type 只能是 MOTOR_TYPE_MF9025 */
typedef struct
{
    can_id_e         bus;
    motor_type_e     type;
    uint8_t          id;                 /* 电机 ID 1..32：命令帧和反馈帧 ID 都是 0x140 + id */
    lk_motor_mode_e  mode;
    bool             reverse;            /* true：反馈角度、速度、转矩取反，输出也取反 */
    uint32_t         timeout_ms;         /* 超过这么久没收到反馈 = 离线，不能为 0 */
    int16_t          iq_limit;           /* 仅 LK_MOTOR_MODE_SPEED：转矩电流上限 1..2048；TORQUE 填 0 */
    const motor_control_config_t *control;   /* 软件控制配置（模式和 PID），NULL = 只用 lk_motor_set_cmd 或硬件闭环 */
} lk_motor_config_t;

typedef struct
{
    motor_t          base;               /* 必须是第一个成员：公共绑定信息和通用反馈 */
    uint8_t          id;
    lk_motor_mode_e  mode;
    int16_t          iq_limit;
    uint32_t         encoder_counts;     /* 编码器一圈的计数，按型号确定（mf9025 为 65536） */
    /* 待发送的控制量（电机方向，已处理 reverse）：由 lk_motor_compute 计算，lk_motor_send 按模式编码发出 */
    int16_t          cmd;                /* TORQUE：iq 原始值，已限幅 */
    bool             cmd_speed_valid;    /* SPEED：本周期算出了有效的速度给定，false 时发 iq = 0 */
    int32_t          cmd_speed;          /* SPEED：速度给定，0.01 dps/LSB */
    uint8_t          poll_count;
    /* 反馈：在事件任务 lk_motor_task 里更新（中断只拷贝帧） */
    uint16_t         encoder;            /* 单圈编码器值 0..encoder_counts-1 */
    uint16_t         last_encoder;
    int32_t          round_count;
    int16_t          speed_dps;
    int16_t          current;            /* 反馈的转矩电流 iq 原始值（未应用 reverse） */
    float            voltage;            /* 母线电压 V（状态 1） */
    float            bus_current;        /* 母线电流 A（状态 1） */
    uint8_t          state;              /* LK_MOTOR_STATE_ON / LK_MOTOR_STATE_OFF（状态 1） */
    uint8_t          error_state;        /* LK_MOTOR_ERROR_xxx（状态 1） */
} lk_motor_t;

/* 在 INIT_APP_STAGE1 里调用（或更早），只做绑定（订阅 0x141..0x160 的反馈帧），不发送任何帧。
 * 参数非法返回 ERR_INVAL，id/总线冲突返回 ERR_BUSY，数量超限返回 ERR_NOMEM，CAN 订阅失败返回对应错误码 */
int32_t lk_motor_register(lk_motor_t *motor, const lk_motor_config_t *config);

/* 直接给 iq 原始值，绕过软件控制。仅对 TORQUE 模式、control == NULL 的电机生效 */
void    lk_motor_set_cmd(lk_motor_t *motor, int16_t cmd);

/* 下面四个各发一帧命令帧。返回 bsp_can_send 的结果：TX FIFO 满为 ERR_BUSY。
 * 电机上电默认就是运行状态；disable 之后再发控制帧即可恢复，enable 不是必需的。
 * 协议里的 0x80 关闭会清圈数和控制指令，这里不提供 */
int32_t lk_motor_enable(lk_motor_t *motor);          /* 0x88 运行 */
int32_t lk_motor_disable(lk_motor_t *motor);         /* 0x81 停止，不清运行状态 */
int32_t lk_motor_set_zero(lk_motor_t *motor);        /* 0x95 当前位置设为零点，写 RAM，掉电失效。之后电机进入停止状态 */
int32_t lk_motor_clear_error(lk_motor_t *motor);     /* 0x9B 清错误标志。故障没恢复时清不掉 */

/* 计算：按"硬件模式 x 控制模式"算出待发送的控制量：
 *   TORQUE  OUTPUT/SPEED/ANGLE -> 目标 / 速度环 / 角度环 -> 速度环，加输出前馈写入 output（没有控制配置时什么都不做）
 *   SPEED   SPEED -> 目标 + 速度前馈；ANGLE -> 软件角度环 + 速度前馈；OUTPUT 不支持
 *   active = false、还没收到反馈、不支持的组合：iq = 0
 * 速度环是增量式，同一个电机一个控制周期内只能计算一次 */
void    lk_motor_compute(lk_motor_t *motor);
/* 计算指定总线上的 lk_motor（跳过没有控制配置的 TORQUE 电机），返回计算了几个；总线号非法返回 ERR_INVAL */
int32_t lk_motor_compute_bus(can_id_e bus);

/* 发送：TORQUE 把 output 应用 reverse、限幅后转成 cmd 发 0xA1；SPEED 发 0xA2。电机收到后才回复反馈，所以要周期发 */
int32_t lk_motor_send(lk_motor_t *motor);               /* 返回 bsp_can_send 的结果：TX FIFO 满为 ERR_BUSY */
int32_t lk_motor_send_bus(can_id_e bus);                /* 指定总线上所有 lk_motor 各发一帧，返回成功发出的帧数 */

/* 清零圈数，从下一帧反馈重新累计多圈角度（只改软件计数，不发帧） */
void    lk_motor_zero(lk_motor_t *motor);
#endif
