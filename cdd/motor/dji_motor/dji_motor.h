#ifndef CDD_DJI_MOTOR_H__
#define CDD_DJI_MOTOR_H__
#include "component/common/types.h"
#include "bsp/include/bsp_ids.h"
#include "cdd/motor/motor.h"

/* GM6020 的控制量种类（M3508/M2006 只有电流，忽略这一项）：
 *   电压：旧固件和默认，报文 0x1FF(id1..4) / 0x2FF(id5..7)，范围 ±25000
 *   电流：新固件，报文 0x1FE(id1..4) / 0x2FE(id5..7)，范围 ±16384（对应 ±3 A） */
typedef enum
{
    DJI_GM6020_VOLTAGE,
    DJI_GM6020_CURRENT,
} dji_gm6020_mode_e;

/* 注册配置：type 只能是 MOTOR_TYPE_M3508 / MOTOR_TYPE_M2006 / MOTOR_TYPE_GM6020 */
typedef struct
{
    can_id_e          bus;
    motor_type_e      type;
    uint8_t           id;                /* 拨码/ID：M3508、M2006 为 1..8，GM6020 为 1..7 */
    bool              reverse;           /* true：反馈角度、速度、转矩取反，输出也取反 */
    uint16_t          offset_encoder;    /* 编码器零点偏移，0..8191；反馈角度 = 原始值 - 偏移 */
    uint32_t          timeout_ms;        /* 超过这么久没收到反馈 = 离线，不能为 0 */
    dji_gm6020_mode_e gm6020_mode;       /* 仅 GM6020 使用 */
    const motor_control_config_t *control;   /* 软件控制配置（PID、外部反馈、前馈），NULL = 只能用 dji_motor_set_cmd 直接给原始值 */
} dji_motor_config_t;

typedef struct
{
    motor_t           base;              /* 必须是第一个成员：公共绑定信息和通用反馈 */
    uint8_t           id;
    dji_gm6020_mode_e gm6020_mode;
    int16_t           cmd;               /* 待发送的电流（M3508/M2006）或电压/电流（GM6020）原始值，已限幅、已处理 reverse */
    /* 反馈：在事件任务 dji_task 里更新（中断只拷贝帧） */
    uint16_t offset_encoder;
    uint16_t encoder;                    /* 减去零点偏移后的单圈编码器值 0..8191 */
    uint16_t last_encoder;
    int32_t  round_count;
    int16_t  speed_rpm;
    int16_t  current;                    /* 反馈的实际电流/转矩原始值（未应用 reverse） */
} dji_motor_t;

/* 在 INIT_APP 阶段之前/之内调用。参数非法返回 ERR_INVAL，id/总线冲突返回 ERR_BUSY，
 * 数量超限返回 ERR_NOMEM，CAN 订阅失败返回对应错误码 */
int32_t dji_motor_register(dji_motor_t *motor, const dji_motor_config_t *config);

/* 直接给原始值（电流/电压），绕过软件控制。已处理 reverse 以外的一切由调用者负责；仅对 control == NULL 的电机生效 */
void    dji_motor_set_cmd(dji_motor_t *motor, int16_t cmd);

/* 对所有已注册、有控制配置的 DJI 电机计算 cmd：
 *   TORQUE：给定 + 转矩前馈   SPEED：速度环 + 转矩前馈   ANGLE：角度环 -> 速度环 + 转矩前馈
 *   然后应用 reverse、按电机型号限幅。active = false 的电机 cmd = 0 */
void    dji_motor_compute_all(void);

/* 把所有已注册 DJI 电机的 cmd 按"总线 x 控制帧"打包发出；没有注册任何电机的帧不发。返回发出的帧数 */
int32_t dji_motor_send_all(void);

/* 清零圈数，从下一帧反馈重新累计多圈角度 */
void    dji_motor_zero(dji_motor_t *motor);
#endif
