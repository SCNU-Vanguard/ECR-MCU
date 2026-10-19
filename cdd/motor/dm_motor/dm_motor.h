#ifndef CDD_DM_MOTOR_H__
#define CDD_DM_MOTOR_H__
#include "component/common/types.h"
#include "bsp/include/bsp_ids.h"
#include "cdd/motor/motor.h"

/* dm_motor（达妙 j4310 / j8009p / s3519）的硬件控制模式。除 TORQUE 外，枚举值就是控制帧 ID 相对 tx_id 的偏移，
 * 要和电机上位机里设置的控制模式一致。
 *   MIT：          ID = tx_id        运控：p_des、v_des、Kp、Kd、t_ff，电机内部 t = Kp(p_des-p) + Kd(v_des-v) + t_ff
 *   TORQUE：       ID = tx_id        仍是 MIT 帧。Kp = Kd = 0，t_ff 为软件级联 PID 的转矩（N·m）。
 *                                    电机上位机里的模式选 MIT，不存在单独的转矩协议模式。
 *                                    转矩经过 motor_t::output，发送前应用层可修改，需要有控制配置
 *   POSITION_SPEED：ID = 0x100+tx_id  位置速度：目标位置 + 速度上限，电机内部闭环
 *   SPEED：         ID = 0x200+tx_id  速度：目标速度，电机内部闭环
 *   FORCE_POSITION：ID = 0x300+tx_id  力位混合：目标位置 + 速度上限 + 电流上限，电机内部闭环 */
typedef enum
{
    DM_MOTOR_MODE_MIT            = 0x000,
    DM_MOTOR_MODE_POSITION_SPEED = 0x100,
    DM_MOTOR_MODE_SPEED          = 0x200,
    DM_MOTOR_MODE_FORCE_POSITION = 0x300,
    DM_MOTOR_MODE_TORQUE         = 0x001,   /* 不是 ID 偏移。控制帧和使能帧的 ID 都按 MIT 计算，即 tx_id */
} dm_motor_mode_e;

/* 状态码：反馈帧里的 state，0 = 失能，1 = 使能，其余都是故障码 */
#define DM_MOTOR_STATE_ENABLED    1u

/* 注册配置：type 只能是 MOTOR_TYPE_J4310 / MOTOR_TYPE_J8009P / MOTOR_TYPE_S3519。
 * p_max / v_max / t_max 必须和电机里设置的映射范围一致，否则反馈和 MIT 控制帧会解码/编码错（4310 常见 12.5 / 30 / 10）。
 * s3519（驱动器型号 3520-1EC）协议与上面相同，但位置是减速前的电机侧，速度和转矩是输出轴。
 * 驱动按减速比 3591/187 把位置换成输出轴 rad，应用层看到的角度、速度、转矩都在输出轴上。
 * 输出轴额定 3.5 N·m / 395 rpm，峰值 7.8 N·m / 空载 435 rpm。 */
typedef struct
{
    can_id_e       bus;
    motor_type_e   type;
    uint8_t        tx_id;            /* 电机的 CAN ID（控制帧 ID 的基数），1..255 */
    uint16_t       master_id;        /* 电机的 Master ID：电机发回反馈帧用的标准帧 ID，同一总线上不能重复 */
    dm_motor_mode_e      mode;
    bool           reverse;          /* true：反馈角度、速度、转矩取反，发出的目标也取反 */
    float          p_max;            /* 位置映射范围 ±p_max，rad */
    float          v_max;            /* 速度映射范围 ±v_max，rad/s */
    float          t_max;            /* 转矩映射范围 ±t_max，N·m */
    uint32_t       timeout_ms;       /* 超过这么久没收到反馈 = 离线，不能为 0 */
    float          kp;               /* 仅 DM_MOTOR_MODE_MIT：电机内部位置刚度，0..500 */
    float          kd;               /* 仅 DM_MOTOR_MODE_MIT：电机内部阻尼，0..5 */
    float          speed_limit;      /* POSITION_SPEED / FORCE_POSITION 的速度上限，rad/s（力位混合 0..100） */
    float          current_limit;    /* FORCE_POSITION 的电流上限，标幺值 0..1（相对电机最大电流） */
    const motor_control_config_t *control;   /* 软件控制配置，NULL = 只用硬件闭环的组合 */
} dm_motor_config_t;

typedef struct
{
    motor_t        base;             /* 必须是第一个成员：公共绑定信息和通用反馈 */
    uint8_t        tx_id;
    uint16_t       master_id;
    dm_motor_mode_e      mode;
    float          p_max;
    float          v_max;
    float          t_max;
    float          kp;
    float          kd;
    float          speed_limit;
    float          current_limit;
    float          gear_ratio;      /* 电机侧 / 输出轴。直驱型号为 1；s3519 为 3591/187 */
    /* 待发送的控制量（电机方向，已处理 reverse）：由 dm_motor_compute 计算，dm_motor_send 按模式编码发出 */
    float          cmd_position;
    float          cmd_velocity;
    float          cmd_kp;
    float          cmd_kd;
    float          cmd_torque;
    float          cmd_current_limit;
    bool           cmd_from_output;  /* true：cmd_torque 在发送时取自 base.output（软件转矩，发送前应用层可改） */
    /* 反馈：在事件任务 dm_motor_task 里更新（中断只拷贝帧） */
    uint8_t        state;            /* 电机状态码：0 失能 1 使能 8 超压 9 欠压 A 过流 B MOS过温 C 线圈过温 D 通信丢失 E 过载 */
    float          position;         /* 协议位置，rad，未应用 reverse。直驱型号即输出轴；s3519 是减速前的电机侧 */
    float          last_position;
    int32_t        round_count;      /* 位置跨过 ±pi 边界的次数，用于累计多圈角度 */
    float          velocity;         /* rad/s（未应用 reverse） */
    float          torque_nm;        /* N·m（未应用 reverse） */
    float          mos_temperature;  /* MOS 管温度，摄氏度 */
} dm_motor_t;

/* 在 INIT_APP_STAGE1 里调用（或更早），只做绑定（订阅 master_id 的反馈帧），不发送任何帧。
 * 参数非法返回 ERR_INVAL，master_id/总线冲突返回 ERR_BUSY，数量超限返回 ERR_NOMEM */
int32_t dm_motor_register(dm_motor_t *motor, const dm_motor_config_t *config);

/* 下面四个各发一帧命令帧（ID 与该模式的控制帧相同；TORQUE 用 MIT 的 tx_id）。返回 bsp_can_send 的结果：TX FIFO 满为 ERR_BUSY */
int32_t dm_motor_enable(dm_motor_t *motor);          /* 使能：之后才响应控制帧 */
int32_t dm_motor_disable(dm_motor_t *motor);         /* 失能 */
int32_t dm_motor_set_zero(dm_motor_t *motor);        /* 把当前位置设为零点 */
int32_t dm_motor_clear_error(dm_motor_t *motor);     /* 清除故障（如过热） */

/* 计算：按"硬件模式 x 控制模式"算出待发送的控制量（cmd_xxx）：
 *   MIT            TORQUE/SPEED/ANGLE -> t_ff / v_des+Kd / p_des+Kp+Kd（Kp、Kd 用配置里的值，电机内部闭环）
 *   TORQUE         级联 PID -> t_ff（Kp = Kd = 0），转矩放在 base.output 里，发送前应用层可以改它
 *   POSITION_SPEED ANGLE -> p_des + speed_limit
 *   SPEED          SPEED -> v_des；ANGLE -> 软件角度环输出 v_des
 *   FORCE_POSITION ANGLE -> p_des + speed_limit + current_limit
 *   不支持的组合、active = false：保持帧（不出力，位置保持在当前位置）
 * 速度环是增量式，同一个电机一个控制周期内只能计算一次 */
void    dm_motor_compute(dm_motor_t *motor);
/* 计算指定总线上所有 dm_motor，返回计算了几个；总线号非法返回 ERR_INVAL */
int32_t dm_motor_compute_bus(can_id_e bus);

/* 发送：按电机的硬件模式编码并发出一帧控制帧（电机收到后才回复反馈，所以要周期发） */
int32_t dm_motor_send(dm_motor_t *motor);               /* 返回 bsp_can_send 的结果：TX FIFO 满为 ERR_BUSY */
int32_t dm_motor_send_bus(can_id_e bus);                /* 指定总线上所有 dm_motor 各发一帧，返回成功发出的帧数 */
#endif
