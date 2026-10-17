#include <string.h>
#include "cdd/motor/dm_motor/dm_motor.h"
#include "bsp/can/bsp_can.h"
#include "bsp/tick/bsp_tick.h"
#include "component/common/err.h"
#include "component/log/log.h"
#include "osal/osal.h"
#include "osal/event.h"

/* DM 协议（依据 DM-J4310-2EC 说明书，与 2026 步兵、标准机器人的报文一致）。
 *
 * 反馈帧（四种模式相同），8 字节：
 *   byte0     高 4 位 = 状态码，低 4 位 = 电机 CAN ID
 *   byte1..2  位置，16 位，映射到 [-p_max, p_max]
 *   byte3..4  速度，12 位，映射到 [-v_max, v_max]（byte3 为高 8 位，byte4 高 4 位为低 4 位）
 *   byte4..5  转矩，12 位，映射到 [-t_max, t_max]（byte4 低 4 位为高 4 位）
 *   byte6     MOS 管温度，byte7 转子温度
 *   反馈是问询式的：电机收到控制帧后才回复，所以必须周期发控制帧。
 *
 * 命令帧：7 个 0xFF 加一个命令字节：0xFC 使能，0xFD 失能，0xFE 设置零点，0xFB 清除故障。
 *
 * 控制帧（ID = 模式偏移 + tx_id）：
 *   MIT 0x000：p_des(16) v_des(12) Kp(12) Kd(12) t_ff(12) 依次位拼接，共 8 字节
 *              p_des ±p_max，v_des ±v_max，Kp 0..500，Kd 0..5，t_ff ±t_max
 *   位置速度 0x100：p_des(float 小端) v_des(float 小端)，8 字节
 *   速度 0x200：v_des(float 小端)，4 字节
 *   力位混合 0x300：p_des(float 小端) v_des(u16 小端，rad/s x100，0..10000) i_des(u16 小端，标幺 x10000，0..10000)，8 字节 */
#define DM_COMMAND_ENABLE       0xFCu
#define DM_COMMAND_DISABLE      0xFDu
#define DM_COMMAND_SET_ZERO     0xFEu
#define DM_COMMAND_CLEAR_ERROR  0xFBu
#define DM_STATE_FAULT_MIN      3u                 /* 状态码 0 失能 1 使能，其余都是故障码 */
#define DM_MAX_STANDARD_ID      0x7FFu
#define DM_KP_MAX               500.0f
#define DM_KD_MAX               5.0f
#define PI                      3.1415927f
#define TWO_PI                  6.2831853f
#define HYBRID_SPEED_SCALE      100.0f
#define HYBRID_CURRENT_SCALE    10000.0f
#define HYBRID_FIELD_MAX        10000.0f
/* DM3519 说明书：齿轮减速比 3591/187（约 1:19.2）。位置在电机侧，速度和转矩在输出轴 */
#define DM3519_GEAR_RATIO     (3591.0f / 187.0f)

static bool is_dm_type(motor_type_e type)
{
    return type == MOTOR_TYPE_DM4310 || type == MOTOR_TYPE_DM8009P || type == MOTOR_TYPE_DM3519;
}

static float gear_ratio_of(motor_type_e type)
{
    return type == MOTOR_TYPE_DM3519 ? DM3519_GEAR_RATIO : 1.0f;
}

static float limit_value(float value, float minimum, float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

static float unsigned_to_float(uint32_t value, float range_min, float range_max, uint32_t bits)
{
    float span = range_max - range_min;

    return (float)value * span / (float)((1u << bits) - 1u) + range_min;
}

static uint32_t float_to_unsigned(float value, float range_min, float range_max, uint32_t bits)
{
    float span = range_max - range_min;

    value = limit_value(value, range_min, range_max);
    return (uint32_t)((value - range_min) * (float)((1u << bits) - 1u) / span);
}

static void update_feedback(dm_motor_t *motor, const can_event_t *event)
{
    uint16_t position_raw = (uint16_t)((event->data[1] << 8) | event->data[2]);
    uint16_t velocity_raw = (uint16_t)((event->data[3] << 4) | (event->data[4] >> 4));
    uint16_t torque_raw = (uint16_t)(((event->data[4] & 0x0F) << 8) | event->data[5]);

    motor->state = (uint8_t)(event->data[0] >> 4);
    motor->last_position = motor->position;
    motor->position = unsigned_to_float(position_raw, -motor->p_max, motor->p_max, 16);
    motor->velocity = unsigned_to_float(velocity_raw, -motor->v_max, motor->v_max, 12);
    motor->torque_nm = unsigned_to_float(torque_raw, -motor->t_max, motor->t_max, 12);
    motor->mos_temperature = (float)event->data[6];

    if (!motor->base.got_first)
    {
        motor->base.got_first = true;
    }
    else
    {
        float position_delta = motor->position - motor->last_position;

        if (position_delta > PI)
        {
            motor->round_count--;                           /* 反转跨过 -pi 边界 */
        }
        else if (position_delta < -PI)
        {
            motor->round_count++;                           /* 正转跨过 +pi 边界 */
        }
    }

    float sign = motor->base.reverse ? -1.0f : 1.0f;
    float output_position = motor->position / motor->gear_ratio;

    motor->base.single_angle_rad = sign * output_position;
    motor->base.total_angle_rad = sign * (motor->position + (float)motor->round_count * TWO_PI) / motor->gear_ratio;
    motor->base.speed_rad_s = sign * motor->velocity;
    motor->base.torque = sign * motor->torque_nm;
    motor->base.temperature = (float)event->data[7];
    if (motor->state >= DM_STATE_FAULT_MIN)
    {
        motor->base.error_flags |= MOTOR_ERROR_FAULT;
    }
    else
    {
        motor->base.error_flags &= ~MOTOR_ERROR_FAULT;
    }
    motor->base.rx_ms = bsp_tick_ms();
}

/* 事件任务：CAN 中断把各 DM 电机 master_id 的帧拷成 can_event_t 投递过来，这里解码 */
static void dm_run(void *argument, const uint8_t *message, uint16_t len)
{
    can_event_t event;

    (void)argument;
    if (message == NULL || len != sizeof event)
    {
        return;
    }
    memcpy(&event, message, sizeof event);
    if (motor_candump_enabled())
    {
        LOGI("dm can%u %03X [%u]", (unsigned)event.bus + 1u, (unsigned)event.id, (unsigned)event.len);
    }
    if (event.len < 8u)
    {
        return;
    }
    for (uint8_t motor_index = 0; ; motor_index++)
    {
        motor_t *base = motor_registry_get(motor_index);

        if (base == NULL)
        {
            return;
        }
        if (!is_dm_type(base->type) || base->bus != (can_id_e)event.bus || base->feedback_id != event.id)
        {
            continue;
        }
        uint32_t irq_key = os_irq_save();                   /* 读者在别的任务里，几个字段要一起更新 */

        update_feedback((dm_motor_t *)base, &event);
        os_irq_restore(irq_key);
        return;
    }
}
EVENT_TASK_DEFINE(dm_motor_task, OS_PRIORITY_HIGHEST, OS_STACK_MEDIUM, EVENT_QUEUE, sizeof(can_event_t), OS_QUEUE_LEN_32, OS_TIMEOUT_NONE, dm_run, NULL);

int32_t dm_motor_register(dm_motor_t *motor, const dm_motor_config_t *config)
{
    if (motor == NULL || config == NULL || !is_dm_type(config->type) ||
        config->tx_id == 0u || config->master_id == 0u || config->master_id > DM_MAX_STANDARD_ID ||
        config->p_max <= 0.0f || config->v_max <= 0.0f || config->t_max <= 0.0f)
    {
        return ERR_INVAL;
    }
    switch (config->mode)
    {
        case DM_MODE_MIT:
        case DM_MODE_POSITION_SPEED:
        case DM_MODE_SPEED:
        case DM_MODE_FORCE_POSITION:
            break;
        default:
            return ERR_INVAL;
    }
    memset(motor, 0, sizeof *motor);
    motor->base.bus = config->bus;
    motor->base.type = config->type;
    motor->base.feedback_id = config->master_id;
    motor->base.reverse = config->reverse;
    motor->base.timeout_ms = config->timeout_ms;
    motor->tx_id = config->tx_id;
    motor->master_id = config->master_id;
    motor->mode = config->mode;
    motor->mit_style = config->mit_style;
    motor->p_max = config->p_max;
    motor->v_max = config->v_max;
    motor->t_max = config->t_max;
    motor->kp = config->kp;
    motor->kd = config->kd;
    motor->speed_limit = config->speed_limit;
    motor->current_limit = config->current_limit;
    motor->gear_ratio = gear_ratio_of(config->type);
    motor_control_setup(&motor->base, config->control);

    int32_t add_ret = motor_registry_add(&motor->base);

    if (add_ret != ERR_OK)
    {
        return add_ret;
    }
    return bsp_can_subscribe_event(config->bus, config->master_id, config->master_id, &dm_motor_task);
}

static int32_t send_command(dm_motor_t *motor, uint8_t command)
{
    can_frame_t frame;

    frame.id = (uint32_t)motor->mode + motor->tx_id;
    frame.len = 8;
    memset(frame.data, 0xFF, 7);
    frame.data[7] = command;
    return bsp_can_send(motor->base.bus, &frame);
}

int32_t dm_motor_enable(dm_motor_t *motor)
{
    return send_command(motor, DM_COMMAND_ENABLE);
}

int32_t dm_motor_disable(dm_motor_t *motor)
{
    return send_command(motor, DM_COMMAND_DISABLE);
}

int32_t dm_motor_set_zero(dm_motor_t *motor)
{
    return send_command(motor, DM_COMMAND_SET_ZERO);
}

int32_t dm_motor_clear_error(dm_motor_t *motor)
{
    return send_command(motor, DM_COMMAND_CLEAR_ERROR);
}

/* ---- 控制量计算 ---- */

/* 保持帧：不出力，位置保持在当前位置（Kp = Kd = 0，速度/电流上限 = 0） */
static void set_hold(dm_motor_t *motor)
{
    motor->cmd_position = motor->position;
    motor->cmd_velocity = 0.0f;
    motor->cmd_kp = 0.0f;
    motor->cmd_kd = 0.0f;
    motor->cmd_torque = 0.0f;
    motor->cmd_current_limit = 0.0f;
    motor->cmd_from_output = false;
}

/* 目标角度回绕到输出轴 [-pi, pi]，再乘减速比换成电机侧位置，并限制在映射范围里 */
static float angle_target_to_motor(const dm_motor_t *motor)
{
    float target = pid_wrap_pi(motor->base.target) * motor->gear_ratio;
    float sign = motor->base.reverse ? -1.0f : 1.0f;

    return sign * limit_value(target, -motor->p_max, motor->p_max);
}

static void compute_mit(dm_motor_t *motor)
{
    motor_t *base = &motor->base;
    float sign = base->reverse ? -1.0f : 1.0f;

    set_hold(motor);
    if (motor->mit_style == DM_MIT_SOFTWARE_TORQUE)
    {
        if (!base->has_control)
        {
            return;
        }
        float reference = base->target;

        if (base->control_mode == MOTOR_CONTROL_ANGLE)
        {
            reference = motor_angle_loop(base, reference);
        }
        if (base->control_mode != MOTOR_CONTROL_TORQUE)
        {
            reference = motor_speed_loop(base, reference);
        }
        base->output = limit_value(reference + motor_torque_feedforward(base), -motor->t_max, motor->t_max);
        motor->cmd_from_output = true;
        return;
    }
    switch (base->control_mode)                             /* 原生 Kp/Kd */
    {
        case MOTOR_CONTROL_TORQUE:
            motor->cmd_torque = sign * (base->target + motor_torque_feedforward(base));
            break;
        case MOTOR_CONTROL_SPEED:
            motor->cmd_velocity = sign * (base->target + motor_speed_feedforward(base));
            motor->cmd_kd = motor->kd;
            motor->cmd_torque = sign * motor_torque_feedforward(base);
            break;
        case MOTOR_CONTROL_ANGLE:
            motor->cmd_position = angle_target_to_motor(motor);
            motor->cmd_velocity = sign * motor_speed_feedforward(base);
            motor->cmd_kp = motor->kp;
            motor->cmd_kd = motor->kd;
            motor->cmd_torque = sign * motor_torque_feedforward(base);
            break;
    }
}

void dm_motor_compute(dm_motor_t *motor)
{
    motor_t *base = &motor->base;
    float sign = base->reverse ? -1.0f : 1.0f;

    set_hold(motor);
    base->output = 0.0f;
    if (!base->active || !motor_control_ready(base))        /* 没打开输出，或还没收到过反馈：保持帧 */
    {
        return;
    }
    switch (motor->mode)
    {
        case DM_MODE_MIT:
            compute_mit(motor);
            break;
        case DM_MODE_POSITION_SPEED:
            if (base->control_mode == MOTOR_CONTROL_ANGLE)
            {
                motor->cmd_position = angle_target_to_motor(motor);
                motor->cmd_velocity = motor->speed_limit;
            }
            break;
        case DM_MODE_SPEED:
            if (base->control_mode == MOTOR_CONTROL_SPEED)
            {
                motor->cmd_velocity = sign * (base->target + motor_speed_feedforward(base));
            }
            else if (base->control_mode == MOTOR_CONTROL_ANGLE && base->has_control)
            {
                float speed_reference = motor_angle_loop(base, base->target) + motor_speed_feedforward(base);

                motor->cmd_velocity = sign * speed_reference;
            }
            break;
        case DM_MODE_FORCE_POSITION:
            if (base->control_mode == MOTOR_CONTROL_ANGLE)
            {
                motor->cmd_position = angle_target_to_motor(motor);
                motor->cmd_velocity = motor->speed_limit;
                motor->cmd_current_limit = motor->current_limit;
            }
            break;
    }
}

int32_t dm_motor_compute_bus(can_id_e bus)
{
    int32_t computed_count = 0;

    if ((unsigned)bus >= CAN_NUM)
    {
        return ERR_INVAL;
    }
    for (uint8_t motor_index = 0; ; motor_index++)
    {
        motor_t *base = motor_registry_get(motor_index);

        if (base == NULL)
        {
            break;
        }
        if (!is_dm_type(base->type) || base->bus != bus)
        {
            continue;
        }
        dm_motor_compute((dm_motor_t *)base);
        computed_count++;
    }
    return computed_count;
}
/* ---- 编码并发送 ---- */
static void put_float_little_endian(uint8_t *destination, float value)
{
    memcpy(destination, &value, sizeof value);              /* Cortex-M 小端，与协议一致 */
}

static void put_uint16_little_endian(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value & 0xFF);
    destination[1] = (uint8_t)(value >> 8);
}

static uint16_t hybrid_field(float value, float scale)
{
    return (uint16_t)limit_value(value * scale, 0.0f, HYBRID_FIELD_MAX);
}

static void encode_frame(const dm_motor_t *motor, can_frame_t *frame)
{
    frame->id = (uint32_t)motor->mode + motor->tx_id;
    frame->len = 8;
    memset(frame->data, 0, sizeof frame->data);
    switch (motor->mode)
    {
        case DM_MODE_MIT:
        {
            float torque = motor->cmd_torque;

            if (motor->cmd_from_output)
            {
                torque = motor->base.reverse ? -motor->base.output : motor->base.output;
            }
            uint32_t position_raw = float_to_unsigned(motor->cmd_position, -motor->p_max, motor->p_max, 16);
            uint32_t velocity_raw = float_to_unsigned(motor->cmd_velocity, -motor->v_max, motor->v_max, 12);
            uint32_t kp_raw = float_to_unsigned(motor->cmd_kp, 0.0f, DM_KP_MAX, 12);
            uint32_t kd_raw = float_to_unsigned(motor->cmd_kd, 0.0f, DM_KD_MAX, 12);
            uint32_t torque_raw = float_to_unsigned(torque, -motor->t_max, motor->t_max, 12);

            frame->data[0] = (uint8_t)(position_raw >> 8);
            frame->data[1] = (uint8_t)(position_raw & 0xFF);
            frame->data[2] = (uint8_t)(velocity_raw >> 4);
            frame->data[3] = (uint8_t)(((velocity_raw & 0x0F) << 4) | (kp_raw >> 8));
            frame->data[4] = (uint8_t)(kp_raw & 0xFF);
            frame->data[5] = (uint8_t)(kd_raw >> 4);
            frame->data[6] = (uint8_t)(((kd_raw & 0x0F) << 4) | (torque_raw >> 8));
            frame->data[7] = (uint8_t)(torque_raw & 0xFF);
            break;
        }
        case DM_MODE_POSITION_SPEED:
            put_float_little_endian(&frame->data[0], motor->cmd_position);
            put_float_little_endian(&frame->data[4], motor->cmd_velocity);
            break;
        case DM_MODE_SPEED:
            frame->len = 4;
            put_float_little_endian(&frame->data[0], motor->cmd_velocity);
            break;
        case DM_MODE_FORCE_POSITION:
            put_float_little_endian(&frame->data[0], motor->cmd_position);
            put_uint16_little_endian(&frame->data[4], hybrid_field(motor->cmd_velocity, HYBRID_SPEED_SCALE));
            put_uint16_little_endian(&frame->data[6], hybrid_field(motor->cmd_current_limit, HYBRID_CURRENT_SCALE));
            break;
    }
}

int32_t dm_motor_send(dm_motor_t *motor)
{
    can_frame_t frame;

    if (motor == NULL)
    {
        return ERR_INVAL;
    }
    encode_frame(motor, &frame);
    return bsp_can_send(motor->base.bus, &frame);
}

int32_t dm_motor_send_bus(can_id_e bus)
{
    int32_t sent_count = 0;

    if ((unsigned)bus >= CAN_NUM)
    {
        return ERR_INVAL;
    }
    for (uint8_t motor_index = 0; ; motor_index++)
    {
        motor_t *base = motor_registry_get(motor_index);

        if (base == NULL)
        {
            break;
        }
        if (!is_dm_type(base->type) || base->bus != bus)
        {
            continue;
        }
        if (dm_motor_send((dm_motor_t *)base) == ERR_OK)
        {
            sent_count++;
        }
    }
    return sent_count;
}