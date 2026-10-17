#include <string.h>
#include "cdd/motor/dji_motor/dji_motor.h"
#include "bsp/can/bsp_can.h"
#include "bsp/tick/bsp_tick.h"
#include "component/common/err.h"
#include "component/log/log.h"
#include "osal/osal.h"
#include "osal/event.h"

/* DJI 协议（依据 C620 / GM6020 官方说明书，与 2026 步兵、标准机器人的报文一致）：
 *   反馈帧 ID：M3508/M2006 = 0x200 + id；GM6020 = 0x204 + id；8 字节，大端：
 *              转子角度(2, 0..8191) 转速rpm(2) 电流/转矩(2) 温度(1) 保留(1)
 *   控制帧：   4 个 int16 大端，对应帧内 4 个电机位置：
 *              M3508/M2006 电流：id1..4 -> 0x200，id5..8 -> 0x1FF
 *              GM6020 电压：     id1..4 -> 0x1FF，id5..7 -> 0x2FF
 *              GM6020 电流(新固件)：id1..4 -> 0x1FE，id5..7 -> 0x2FE
 *   控制量范围：M3508 ±16384（±20 A）；M2006 ±10000（±10 A，C610 手册）；GM6020 电压 ±25000，电流 ±16384（±3 A）
 * 注意：同一条总线上 M3508 的 id5..8 与 GM6020 的 id1..4 反馈 ID/控制帧 ID 重叠，不能混接，
 *       重叠时 motor_registry_add 会因为反馈 ID 相同返回 ERR_BUSY。 */
#define FB_ID_MIN           0x201u
#define FB_ID_MAX           0x20Bu
#define ENCODER_COUNTS      8192
#define TWO_PI              6.2831853f
#define PI                  3.1415927f
#define RPM_TO_RAD_PER_SEC  0.10471976f           /* 2 * pi / 60 */

#define TX_FRAME_COUNT      5                      /* 一条总线上最多 5 种控制帧 */
static const uint16_t tx_ids[TX_FRAME_COUNT] = { 0x200, 0x1FF, 0x2FF, 0x1FE, 0x2FE };

#define LIMIT_M3508         16384.0f
#define LIMIT_M2006         10000.0f
#define LIMIT_GM6020_VOLT   25000.0f
#define LIMIT_GM6020_CURR   16384.0f

static bool subscribed[CAN_NUM];

static bool is_dji_type(motor_type_e type)
{
    return type == MOTOR_TYPE_M3508 || type == MOTOR_TYPE_M2006 || type == MOTOR_TYPE_GM6020;
}

static uint16_t feedback_id_of(motor_type_e type, uint8_t id)
{
    return (uint16_t)((type == MOTOR_TYPE_GM6020 ? 0x204u : 0x200u) + id);
}

/* 控制帧：返回 tx_ids 下标，*slot = 帧内位置 0..3；非法返回 -1 */
static int tx_slot(motor_type_e type, dji_gm6020_mode_e gm6020_mode, uint8_t id, uint8_t *slot)
{
    if (id == 0u)
    {
        return -1;
    }
    if (type == MOTOR_TYPE_GM6020)
    {
        if (id > 7u)
        {
            return -1;
        }
        *slot = (uint8_t)((id - 1u) & 3u);
        if (gm6020_mode == DJI_GM6020_CURRENT)
        {
            return (id <= 4u) ? 3 : 4;
        }
        return (id <= 4u) ? 1 : 2;
    }
    if (id > 8u)
    {
        return -1;
    }
    *slot = (uint8_t)((id - 1u) & 3u);
    return (id <= 4u) ? 0 : 1;
}

static float output_limit_of(const dji_motor_t *motor)
{
    switch (motor->base.type)
    {
        case MOTOR_TYPE_GM6020:
            return (motor->gm6020_mode == DJI_GM6020_CURRENT) ? LIMIT_GM6020_CURR : LIMIT_GM6020_VOLT;
        case MOTOR_TYPE_M2006:
            return LIMIT_M2006;
        default:
            return LIMIT_M3508;
    }
}

static int16_t to_command(float value, float limit)
{
    if (value > limit)
    {
        value = limit;
    }
    if (value < -limit)
    {
        value = -limit;
    }
    return (int16_t)value;
}

static void update_feedback(dji_motor_t *motor, const can_event_t *event)
{
    uint16_t raw_encoder = (uint16_t)((event->data[0] << 8) | event->data[1]);
    uint16_t encoder = (raw_encoder >= motor->offset_encoder)
                           ? (uint16_t)(raw_encoder - motor->offset_encoder)
                           : (uint16_t)(raw_encoder + ENCODER_COUNTS - motor->offset_encoder);

    if (!motor->base.got_first)
    {
        motor->base.got_first = true;
    }
    else
    {
        int encoder_delta = (int)encoder - (int)motor->last_encoder;

        if (encoder_delta > ENCODER_COUNTS / 2)
        {
            motor->round_count--;                           /* 反转过零点 */
        }
        else if (encoder_delta < -ENCODER_COUNTS / 2)
        {
            motor->round_count++;                           /* 正转过零点 */
        }
    }
    motor->last_encoder = encoder;
    motor->encoder = encoder;
    motor->speed_rpm = (int16_t)((event->data[2] << 8) | event->data[3]);
    motor->current = (int16_t)((event->data[4] << 8) | event->data[5]);

    float sign = motor->base.reverse ? -1.0f : 1.0f;
    float total_encoder = (float)motor->round_count * ENCODER_COUNTS + (float)encoder;
    float single_angle = (float)encoder * (TWO_PI / ENCODER_COUNTS);

    if (single_angle >= PI)                                 /* 映射到 [-pi, pi) */
    {
        single_angle -= TWO_PI;
    }
    motor->base.single_angle_rad = sign * single_angle;
    motor->base.total_angle_rad = sign * total_encoder * (TWO_PI / ENCODER_COUNTS);
    motor->base.speed_rad_s = sign * (float)motor->speed_rpm * RPM_TO_RAD_PER_SEC;
    motor->base.torque = sign * (float)motor->current;
    motor->base.temperature = (motor->base.type == MOTOR_TYPE_M2006) ? 0.0f : (float)event->data[6];   /* C610 反馈没有温度字节 */
    motor->base.rx_ms = bsp_tick_ms();
}

/* 事件任务：CAN 中断把 0x201..0x20B 的帧拷成 can_event_t 投递过来，这里解码。
 * 优先级设为 0（最高）：先于 1 kHz 控制任务更新反馈，控制任务读到的就是最新的 */
static void dji_run(void *argument, const uint8_t *message, uint16_t len)
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
        LOGI("dji can%u %03X [%u]", (unsigned)event.bus + 1u, (unsigned)event.id, (unsigned)event.len);
    }
    if (event.len < 7u)
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
        if (!is_dji_type(base->type) || base->bus != (can_id_e)event.bus || base->feedback_id != event.id)
        {
            continue;
        }
        uint32_t irq_key = os_irq_save();                   /* 读者在别的任务里，几个字段要一起更新 */

        update_feedback((dji_motor_t *)base, &event);
        os_irq_restore(irq_key);
        return;
    }
}
EVENT_TASK_DEFINE(dji_motor_task, OS_PRIORITY_HIGHEST, OS_STACK_MEDIUM, EVENT_QUEUE, sizeof(can_event_t), OS_QUEUE_LEN_32, OS_TIMEOUT_NONE, dji_run, NULL);

int32_t dji_motor_register(dji_motor_t *motor, const dji_motor_config_t *config)
{
    uint8_t slot;

    if (motor == NULL || config == NULL || !is_dji_type(config->type) ||
        config->offset_encoder >= ENCODER_COUNTS || tx_slot(config->type, config->gm6020_mode, config->id, &slot) < 0)
    {
        return ERR_INVAL;
    }
    memset(motor, 0, sizeof *motor);
    motor->base.bus = config->bus;
    motor->base.type = config->type;
    motor->base.feedback_id = feedback_id_of(config->type, config->id);
    motor->base.reverse = config->reverse;
    motor->base.timeout_ms = config->timeout_ms;
    motor->id = config->id;
    motor->gm6020_mode = config->gm6020_mode;
    motor->offset_encoder = config->offset_encoder;
    motor_control_setup(&motor->base, config->control);

    int32_t add_ret = motor_registry_add(&motor->base);

    if (add_ret != ERR_OK)
    {
        return add_ret;
    }
    if (!subscribed[config->bus])
    {
        int32_t subscribe_ret = bsp_can_subscribe_event(config->bus, FB_ID_MIN, FB_ID_MAX, &dji_motor_task);

        if (subscribe_ret != ERR_OK)
        {
            return subscribe_ret;
        }
        subscribed[config->bus] = true;
    }
    return ERR_OK;
}

void dji_motor_set_cmd(dji_motor_t *motor, int16_t cmd)
{
    if (!motor->base.has_control)
    {
        motor->cmd = cmd;
    }
}

void dji_motor_zero(dji_motor_t *motor)
{
    uint32_t irq_key = os_irq_save();

    motor->round_count = 0;
    motor->base.total_angle_rad = 0.0f;
    motor->base.got_first = false;      /* 下一帧重新作为起点 */
    os_irq_restore(irq_key);
}

void dji_motor_compute(dji_motor_t *motor)
{
    motor_t *base = &motor->base;

    if (!base->has_control)
    {
        return;
    }
    if (!base->active)
    {
        base->output = 0.0f;
        motor->cmd = 0;
        return;
    }
    if (!motor_control_ready(base))                         /* 还没收到过反馈：不输出 */
    {
        base->output = 0.0f;
        motor->cmd = 0;
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
    base->output = reference + motor_torque_feedforward(base);
}

int32_t dji_motor_compute_bus(can_id_e bus)
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
        if (!is_dji_type(base->type) || base->bus != bus || !base->has_control)
        {
            continue;
        }
        dji_motor_compute((dji_motor_t *)base);
        computed_count++;
    }
    return computed_count;
}

/* output（用户方向的转矩给定）-> cmd（电机方向、限幅后的原始值）。
 * 放在发送时做，这样应用层在计算和发送之间改 output（比如功率限制）也会生效 */
static void convert_output(dji_motor_t *motor)
{
    motor_t *base = &motor->base;

    if (!base->has_control || !base->active)
    {
        return;
    }
    float physical_output = base->reverse ? -base->output : base->output;

    motor->cmd = to_command(physical_output, output_limit_of(motor));
}

int32_t dji_motor_send_bus(can_id_e bus)
{
    int16_t values[TX_FRAME_COUNT][4];
    bool    used[TX_FRAME_COUNT];

    if ((unsigned)bus >= CAN_NUM)
    {
        return ERR_INVAL;
    }
    memset(values, 0, sizeof values);
    memset(used, 0, sizeof used);

    for (uint8_t motor_index = 0; ; motor_index++)
    {
        motor_t *base = motor_registry_get(motor_index);

        if (base == NULL)
        {
            break;
        }
        if (!is_dji_type(base->type) || base->bus != bus)
        {
            continue;
        }
        dji_motor_t *motor = (dji_motor_t *)base;
        uint8_t slot = 0;
        int frame_index = tx_slot(base->type, motor->gm6020_mode, motor->id, &slot);

        if (frame_index < 0)                                /* register 时已校验，这里只是防御 */
        {
            continue;
        }
        convert_output(motor);
        values[frame_index][slot] = motor->cmd;
        used[frame_index] = true;
    }

    int32_t sent_count = 0;

    for (int frame_index = 0; frame_index < TX_FRAME_COUNT; frame_index++)
    {
        if (!used[frame_index])                             /* 没有电机的帧不发：无应答的帧会被无限重发 */
        {
            continue;
        }
        can_frame_t frame;

        frame.id = tx_ids[frame_index];
        frame.len = 8;
        for (int slot = 0; slot < 4; slot++)
        {
            frame.data[2 * slot]     = (uint8_t)((uint16_t)values[frame_index][slot] >> 8);
            frame.data[2 * slot + 1] = (uint8_t)(values[frame_index][slot] & 0xFF);
        }
        if (bsp_can_send(bus, &frame) == ERR_OK)
        {
            sent_count++;
        }
    }
    return sent_count;
}