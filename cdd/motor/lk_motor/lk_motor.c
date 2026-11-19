#include <string.h>
#include "cdd/motor/lk_motor/lk_motor.h"
#include "bsp/can/bsp_can.h"
#include "bsp/tick/bsp_tick.h"
#include "component/common/err.h"
#include "component/log/log.h"
#include "osal/osal.h"
#include "osal/event.h"

/* lk_motor 协议（依据瓴控《电机 CAN 总线通讯协议》V2.36）：
 *   命令帧 ID = 回复帧 ID = 0x140 + id（1..32），标准帧，DLC 8，多字节字段都是小端。
 *   反馈是问询式的：电机收到命令后 0.25 ms 内回复，所以必须周期发控制帧。
 *   状态 2 布局（0x9C 读状态 2，以及 0xA0..0xA8 所有控制命令的回复）：
 *     DATA[1] 温度 int8 1℃   DATA[2..3] iq int16   DATA[4..5] 转速 int16 1 dps   DATA[6..7] 编码器 uint16
 *   状态 1 布局（0x9A 读状态 1、0x9B 清错误的回复）：
 *     DATA[1] 温度   DATA[2..3] 母线电压 int16 0.01 V   DATA[4..5] 母线电流 int16 0.01 A   DATA[6] motorState   DATA[7] errorState
 *   控制帧：
 *     0xA1 转矩闭环：DATA[4..5] iqControl int16，±2048（mf ±16.5 A）
 *     0xA2 速度闭环：DATA[2..3] iq 上限 int16，DATA[4..7] speedControl int32，0.01 dps/LSB
 *   命令帧（其余字节为 0）：0x88 运行，0x81 停止，0x95 当前位置设为零点（RAM），0x9A 读状态 1，0x9B 清错误 */
#define LK_COMMAND_STOP           0x81u
#define LK_COMMAND_RUN            0x88u
#define LK_COMMAND_SET_ZERO       0x95u
#define LK_COMMAND_READ_STATE1    0x9Au
#define LK_COMMAND_CLEAR_ERROR    0x9Bu
#define LK_COMMAND_READ_STATE2    0x9Cu
#define LK_COMMAND_CONTROL_FIRST  0xA0u          /* 0xA0..0xA8 都是控制命令，回复都是状态 2 布局 */
#define LK_COMMAND_TORQUE         0xA1u
#define LK_COMMAND_SPEED          0xA2u
#define LK_COMMAND_CONTROL_LAST   0xA8u

#define FB_ID_BASE          0x140u
#define FB_ID_MIN           0x141u
#define FB_ID_MAX           0x160u
#define LK_ID_MAX           32u
#define LK_IQ_MAX           2048.0f
#define ENCODER_COUNTS_MF   65536u               /* mf9025 为 16 位编码器 */
#define TWO_PI              6.2831853f
#define PI                  3.1415927f
#define DPS_TO_RAD_PER_SEC  0.017453292f
#define RAD_PER_SEC_TO_CENTI_DPS 5729.5780f      /* 1 rad/s = 57.29578 dps = 5729.578 x 0.01 dps */
#define SPEED_COMMAND_LIMIT 1.0e9f               /* 只防 float 转 int32 溢出，实际上限由上位机 Max Speed 决定 */
#define STATE1_SCALE        0.01f                /* 状态 1：母线电压、母线电流都是 0.01/LSB */

static bool subscribed[CAN_NUM];
static bool lk_motor_dump_enabled;

static uint32_t encoder_counts_of(motor_type_e type)
{
    (void)type;
    return ENCODER_COUNTS_MF;
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

static uint16_t get_uint16_little_endian(const uint8_t *source)
{
    return (uint16_t)((uint16_t)source[0] | ((uint16_t)source[1] << 8));
}

static void put_uint32_little_endian(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value & 0xFF);
    destination[1] = (uint8_t)(value >> 8);
    destination[2] = (uint8_t)(value >> 16);
    destination[3] = (uint8_t)(value >> 24);
}

static void update_feedback(lk_motor_t *motor, const can_event_t *event)
{
    uint16_t encoder = get_uint16_little_endian(&event->data[6]);
    int32_t half_counts = (int32_t)(motor->encoder_counts / 2u);

    if (!motor->base.got_first)
    {
        motor->base.got_first = true;
    }
    else
    {
        int32_t encoder_delta = (int32_t)encoder - (int32_t)motor->last_encoder;

        if (encoder_delta > half_counts)
        {
            motor->round_count--;                           /* 反转过零点 */
        }
        else if (encoder_delta < -half_counts)
        {
            motor->round_count++;                           /* 正转过零点 */
        }
    }
    motor->last_encoder = encoder;
    motor->encoder = encoder;
    motor->current = (int16_t)get_uint16_little_endian(&event->data[2]);
    motor->speed_dps = (int16_t)get_uint16_little_endian(&event->data[4]);

    float sign = motor->base.reverse ? -1.0f : 1.0f;
    float turn = (float)encoder / (float)motor->encoder_counts;
    float single_angle = turn * TWO_PI;

    if (single_angle >= PI)                                 /* 映射到 [-pi, pi) */
    {
        single_angle -= TWO_PI;
    }
    motor->base.single_angle_rad = sign * single_angle;
    motor->base.total_angle_rad = sign * ((float)motor->round_count + turn) * TWO_PI;
    motor->base.speed_rad_s = sign * (float)motor->speed_dps * DPS_TO_RAD_PER_SEC;
    motor->base.torque = sign * (float)motor->current;
    motor->base.temperature = (float)(int8_t)event->data[1];
    motor->base.rx_ms = bsp_tick_ms();
}

/* 只更新状态和故障位，不更新 rx_ms：在线判断只看带角度的状态 2 反馈 */
static void update_state1(lk_motor_t *motor, const can_event_t *event)
{
    motor->voltage = (float)(int16_t)get_uint16_little_endian(&event->data[2]) * STATE1_SCALE;
    motor->bus_current = (float)(int16_t)get_uint16_little_endian(&event->data[4]) * STATE1_SCALE;
    motor->state = event->data[6];
    motor->error_state = event->data[7];
    motor->base.temperature = (float)(int8_t)event->data[1];
    if (motor->error_state != 0u)
    {
        motor->base.error_flags |= MOTOR_ERROR_FAULT;
    }
    else
    {
        motor->base.error_flags &= ~MOTOR_ERROR_FAULT;
    }
}

/* 事件任务：CAN 中断把 0x141..0x160 的帧拷成 can_event_t 投递过来，这里按命令字节解码 */
static void lk_motor_run(void *argument, const uint8_t *message, uint16_t len)
{
    can_event_t event;

    (void)argument;
    if (message == NULL || len != sizeof event)
    {
        return;
    }
    memcpy(&event, message, sizeof event);
    if (lk_motor_dump_enabled)
    {
        LOGI("lk_motor can%u %03X [%u] %02X", (unsigned)event.bus + 1u, (unsigned)event.id, (unsigned)event.len, (unsigned)event.data[0]);
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
        if (base->type != MOTOR_TYPE_MF9025 || base->bus != (can_id_e)event.bus || base->feedback_id != event.id)
        {
            continue;
        }
        uint8_t command = event.data[0];
        uint32_t irq_key = os_irq_save();                   /* 读者在别的任务里，几个字段要一起更新 */

        if (command == LK_COMMAND_READ_STATE2 || (command >= LK_COMMAND_CONTROL_FIRST && command <= LK_COMMAND_CONTROL_LAST))
        {
            update_feedback((lk_motor_t *)base, &event);
        }
        else if (command == LK_COMMAND_READ_STATE1 || command == LK_COMMAND_CLEAR_ERROR)
        {
            update_state1((lk_motor_t *)base, &event);
        }
        os_irq_restore(irq_key);
        return;
    }
}
EVENT_TASK_DEFINE(lk_motor_task, OS_PRIORITY_HIGHEST, OS_STACK_MEDIUM, EVENT_QUEUE, sizeof(can_event_t), OS_QUEUE_LEN_32, OS_TIMEOUT_NONE, lk_motor_run, NULL);

int32_t lk_motor_register(lk_motor_t *motor, const lk_motor_config_t *config)
{
    if (motor == NULL || config == NULL || config->type != MOTOR_TYPE_MF9025 ||
        config->id == 0u || config->id > LK_ID_MAX)
    {
        return ERR_INVAL;
    }
    switch (config->mode)
    {
        case LK_MOTOR_MODE_TORQUE:
            break;
        case LK_MOTOR_MODE_SPEED:
            if (config->iq_limit <= 0 || (float)config->iq_limit > LK_IQ_MAX)
            {
                return ERR_INVAL;
            }
            break;
        default:
            return ERR_INVAL;
    }
    memset(motor, 0, sizeof *motor);
    motor->base.bus = config->bus;
    motor->base.type = config->type;
    motor->base.feedback_id = (uint16_t)(FB_ID_BASE + config->id);
    motor->base.reverse = config->reverse;
    motor->base.timeout_ms = config->timeout_ms;
    motor->id = config->id;
    motor->mode = config->mode;
    motor->iq_limit = config->iq_limit;
    motor->encoder_counts = encoder_counts_of(config->type);
    motor_control_setup(&motor->base, config->control);

    int32_t add_ret = motor_registry_add(&motor->base);    /* 检查总线号和 timeout_ms */

    if (add_ret != ERR_OK)
    {
        return add_ret;
    }
    if (!subscribed[config->bus])
    {
        int32_t subscribe_ret = bsp_can_subscribe_event(config->bus, FB_ID_MIN, FB_ID_MAX, &lk_motor_task);

        if (subscribe_ret != ERR_OK)
        {
            return subscribe_ret;
        }
        subscribed[config->bus] = true;
    }
    return ERR_OK;
}

static int32_t send_command(lk_motor_t *motor, uint8_t command)
{
    can_frame_t frame;

    frame.id = FB_ID_BASE + motor->id;
    frame.len = 8;
    memset(frame.data, 0, sizeof frame.data);
    frame.data[0] = command;
    return bsp_can_send(motor->base.bus, &frame);
}

int32_t lk_motor_enable(lk_motor_t *motor)
{
    return send_command(motor, LK_COMMAND_RUN);
}

int32_t lk_motor_disable(lk_motor_t *motor)
{
    return send_command(motor, LK_COMMAND_STOP);
}

int32_t lk_motor_set_zero(lk_motor_t *motor)
{
    return send_command(motor, LK_COMMAND_SET_ZERO);
}

int32_t lk_motor_clear_error(lk_motor_t *motor)
{
    return send_command(motor, LK_COMMAND_CLEAR_ERROR);
}

void lk_motor_set_cmd(lk_motor_t *motor, int16_t cmd)
{
    if (motor->mode == LK_MOTOR_MODE_TORQUE && !motor->base.has_control)
    {
        motor->cmd = (int16_t)limit_value((float)cmd, -LK_IQ_MAX, LK_IQ_MAX);
    }
}

void lk_motor_zero(lk_motor_t *motor)
{
    uint32_t irq_key = os_irq_save();

    motor->round_count = 0;
    motor->base.total_angle_rad = 0.0f;
    motor->base.got_first = false;      /* 下一帧重新作为起点 */
    os_irq_restore(irq_key);
}

/* ---- 控制量计算 ---- */

/* TORQUE：软件级联，结果写入 output，发送时再应用 reverse、限幅换成 cmd。没有控制配置时保留 lk_motor_set_cmd 给的 cmd */
static void compute_torque(lk_motor_t *motor)
{
    motor_t *base = &motor->base;

    if (!base->has_control)
    {
        return;
    }
    base->output = 0.0f;
    motor->cmd = 0;
    if (!base->active || !motor_control_ready(base))        /* 没打开输出，或还没收到过反馈：iq = 0 */
    {
        return;
    }
    float reference = base->target;

    if (base->control_mode == MOTOR_CONTROL_ANGLE)
    {
        reference = motor_angle_loop(base, reference);
    }
    if (base->control_mode != MOTOR_CONTROL_OUTPUT)
    {
        reference = motor_speed_loop(base, reference);
    }
    base->output = reference + motor_output_feedforward(base);
}

/* SPEED：电机内部速度环。不支持的组合保持 cmd_speed_valid = false，发送时发 iq = 0 */
static void compute_speed(lk_motor_t *motor)
{
    motor_t *base = &motor->base;
    float speed_reference;

    base->output = 0.0f;
    motor->cmd = 0;
    if (!base->active || !motor_control_ready(base))
    {
        return;
    }
    if (base->control_mode == MOTOR_CONTROL_SPEED)
    {
        speed_reference = base->target + motor_speed_feedforward(base);
    }
    else if (base->control_mode == MOTOR_CONTROL_ANGLE && base->has_control)
    {
        speed_reference = motor_angle_loop(base, base->target) + motor_speed_feedforward(base);
    }
    else
    {
        return;
    }
    float sign = base->reverse ? -1.0f : 1.0f;

    motor->cmd_speed = (int32_t)limit_value(sign * speed_reference * RAD_PER_SEC_TO_CENTI_DPS, -SPEED_COMMAND_LIMIT, SPEED_COMMAND_LIMIT);
    motor->cmd_speed_valid = true;
}

void lk_motor_compute(lk_motor_t *motor)
{
    motor->cmd_speed_valid = false;
    switch (motor->mode)
    {
        case LK_MOTOR_MODE_TORQUE:
            compute_torque(motor);
            break;
        case LK_MOTOR_MODE_SPEED:
            compute_speed(motor);
            break;
    }
}

int32_t lk_motor_compute_bus(can_id_e bus)
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
        if (base->type != MOTOR_TYPE_MF9025 || base->bus != bus)
        {
            continue;
        }
        if (((lk_motor_t *)base)->mode == LK_MOTOR_MODE_TORQUE && !base->has_control)
        {
            continue;                                       /* 和 dji_motor 一样：cmd 由 lk_motor_set_cmd 直接给 */
        }
        lk_motor_compute((lk_motor_t *)base);
        computed_count++;
    }
    return computed_count;
}

/* ---- 编码并发送 ---- */
static void encode_frame(lk_motor_t *motor, can_frame_t *frame)
{
    frame->id = FB_ID_BASE + motor->id;
    frame->len = 8;
    memset(frame->data, 0, sizeof frame->data);
    if (motor->mode == LK_MOTOR_MODE_SPEED && motor->cmd_speed_valid)
    {
        frame->data[0] = LK_COMMAND_SPEED;
        frame->data[2] = (uint8_t)((uint16_t)motor->iq_limit & 0xFFu);
        frame->data[3] = (uint8_t)((uint16_t)motor->iq_limit >> 8);
        put_uint32_little_endian(&frame->data[4], (uint32_t)motor->cmd_speed);
        return;
    }
    if (motor->mode == LK_MOTOR_MODE_TORQUE && motor->base.has_control)
    {
        float physical = motor->base.reverse ? -motor->base.output : motor->base.output;

        motor->cmd = motor->base.active ? (int16_t)limit_value(physical, -LK_IQ_MAX, LK_IQ_MAX) : 0;
    }
    frame->data[0] = LK_COMMAND_TORQUE;
    frame->data[4] = (uint8_t)((uint16_t)motor->cmd & 0xFFu);
    frame->data[5] = (uint8_t)((uint16_t)motor->cmd >> 8);             /* SPEED 模式这里 cmd 恒为 0 */
}

int32_t lk_motor_send(lk_motor_t *motor)
{
    can_frame_t frame;

    if (motor == NULL)
    {
        return ERR_INVAL;
    }
    encode_frame(motor, &frame);

    int32_t send_ret = bsp_can_send(motor->base.bus, &frame);

    /* 控制帧没发出去时不追加问询，免得 TX FIFO 更满 */
    if (send_ret == ERR_OK && ++motor->poll_count >= LK_MOTOR_STATE1_POLL_PERIOD)
    {
        motor->poll_count = 0;
        (void)send_command(motor, LK_COMMAND_READ_STATE1);
    }
    return send_ret;
}

int32_t lk_motor_send_bus(can_id_e bus)
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
        if (base->type != MOTOR_TYPE_MF9025 || base->bus != bus)
        {
            continue;
        }
        if (lk_motor_send((lk_motor_t *)base) == ERR_OK)
        {
            sent_count++;
        }
    }
    return sent_count;
}

#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"

static int shell_lk_motor_dump(int argc, char *argv[])
{
    if (argc == 1)
    {
        shell_out("lk_motor_dump %s\r\n", lk_motor_dump_enabled ? "on" : "off");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "on") == 0)
    {
        lk_motor_dump_enabled = true;
    }
    else if (argc == 2 && strcmp(argv[1], "off") == 0)
    {
        lk_motor_dump_enabled = false;
    }
    else
    {
        shell_out("usage: lk_motor_dump [on|off]\r\n");
        return -1;
    }
    shell_out("lk_motor_dump %s\r\n", lk_motor_dump_enabled ? "on" : "off");
    return 0;
}
SHELL_CMD(lk_motor_dump, shell_lk_motor_dump, print_each_received_lk_motor_can_frame)
#endif
