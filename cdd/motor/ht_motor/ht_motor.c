#include <math.h>
#include <string.h>
#include "cdd/motor/ht_motor/ht_motor.h"
#include "bsp/can/bsp_can.h"
#include "bsp/tick/bsp_tick.h"
#include "component/common/err.h"
#include "osal/osal.h"

#define HT_MAX_ID          0x7Eu
#define HT_REPLY_FLAG      0x8000u
#define HT_MIT_ID          0x18000u
#define HT_TWO_PI          6.283185307f
#define HT_POSITION_SCALE  10000.0f
#define HT_VELOCITY_SCALE  4000.0f
#define HT_TORQUE_SCALE    100.0f

static bool subscribed[CAN_NUM];

static bool is_ht_motor(motor_type_e type)
{
    return type == MOTOR_TYPE_3532;
}

static float clamp(float value, float low, float high)
{
    return fminf(fmaxf(value, low), high);
}

static int16_t read_i16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void write_i16(uint8_t *data, float counts)
{
    uint16_t raw = (uint16_t)(int16_t)clamp(counts, -32760.0f, 32760.0f);

    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)(raw >> 8);
}

static uint16_t mit_field(float value, float low, float high, unsigned bits)
{
    float levels = (float)(1u << bits);
    float encoded = (clamp(value, low, high) - low) * levels / (high - low);

    return (uint16_t)fminf(encoded, levels - 1.0f);
}

static void ht_motor_run(void *argument, const uint8_t *message, uint16_t len)
{
    can_event_t event;

    (void)argument;
    if (message == NULL || len != sizeof event)
    {
        return;
    }
    memcpy(&event, message, sizeof event);
    if (event.len != 8u || (event.id & 0xFFu) != 0u)
    {
        return;
    }
    bool query = event.data[0] == 0x27u && event.data[1] == 0x01u;
    if (!query && event.data[0] == 0x27u)
    {
        return;
    }
    int16_t position = read_i16(&event.data[2]);
    int16_t velocity = read_i16(&event.data[4]);
    int16_t torque = read_i16(&event.data[6]);
    if (position == INT16_MIN || velocity == INT16_MIN || torque == INT16_MIN)
    {
        return;
    }
    for (uint8_t index = 0; ; index++)
    {
        motor_t *base = motor_registry_get(index);
        if (base == NULL)
        {
            return;
        }
        if (!is_ht_motor(base->type) || base->bus != (can_id_e)event.bus || base->feedback_id != event.id)
        {
            continue;
        }
        ht_motor_t *motor = (ht_motor_t *)base;
        float sign = base->reverse ? -1.0f : 1.0f;
        float angle = sign * (float)position * (HT_TWO_PI / HT_POSITION_SCALE);
        uint32_t irq_key = os_irq_save();

        base->total_angle_rad = angle;
        base->single_angle_rad = pid_wrap_pi(angle);
        base->speed_rad_s = sign * (float)velocity * (HT_TWO_PI / HT_VELOCITY_SCALE);
        base->torque = sign * (float)torque * motor->config.torque_scale / HT_TORQUE_SCALE;
        if (!query)
        {
            motor->state = event.data[0];
            motor->fault = event.data[1];
            if (motor->fault != 0u)
            {
                base->error_flags |= MOTOR_ERROR_FAULT;
            }
            else
            {
                base->error_flags &= ~MOTOR_ERROR_FAULT;
            }
        }
        base->rx_ms = bsp_tick_ms();
        base->got_first = true;
        os_irq_restore(irq_key);
        return;
    }
}

EVENT_TASK_DEFINE(ht_motor_task, OS_PRIORITY_HIGHEST, OS_STACK_MEDIUM, EVENT_QUEUE,
                  sizeof(can_event_t), OS_QUEUE_LEN_32, OS_TIMEOUT_NONE, ht_motor_run, NULL);

int32_t ht_motor_register(ht_motor_t *motor, const ht_motor_config_t *config)
{
    if (motor == NULL || config == NULL || !is_ht_motor(config->type) ||
        (unsigned)config->bus >= CAN_NUM || config->timeout_ms == 0u ||
        config->id == 0u || config->id > HT_MAX_ID ||
        (unsigned)config->mode > HT_MOTOR_MODE_MIT ||
        !isfinite(config->torque_scale) || config->torque_scale <= 0.0f ||
        !isfinite(config->torque_limit) || config->torque_limit <= 0.0f)
    {
        return ERR_INVAL;
    }
    if (config->mode == HT_MOTOR_MODE_POSITION_SPEED &&
        (!isfinite(config->speed_limit) || config->speed_limit <= 0.0f ||
         config->control == NULL || config->control->mode != MOTOR_CONTROL_ANGLE))
    {
        return ERR_INVAL;
    }
    if (config->mode == HT_MOTOR_MODE_MIT &&
        (!isfinite(config->kp) || config->kp < 0.0f || !isfinite(config->kd) || config->kd < 0.0f))
    {
        return ERR_INVAL;
    }
    if (!subscribed[config->bus])
    {
        int32_t ret = bsp_can_subscribe_event(config->bus, 0x100u, HT_MAX_ID << 8, &ht_motor_task);
        if (ret != ERR_OK)
        {
            return ret;
        }
        subscribed[config->bus] = true;
    }
    memset(motor, 0, sizeof *motor);
    motor->config = *config;
    motor->base.bus = config->bus;
    motor->base.type = config->type;
    motor->base.feedback_id = (uint16_t)config->id << 8;
    motor->base.reverse = config->reverse;
    motor->base.timeout_ms = config->timeout_ms;
    motor_control_setup(&motor->base, config->control);
    return motor_registry_add(&motor->base);
}

void ht_motor_compute(ht_motor_t *motor)
{
    motor_t *base = &motor->base;
    float sign = base->reverse ? -1.0f : 1.0f;

    motor->cmd_valid = false;
    motor->cmd_position = 0.0f;
    motor->cmd_velocity = 0.0f;
    motor->cmd_kp = 0.0f;
    motor->cmd_kd = 0.0f;
    base->output = 0.0f;
    if (!base->active || !motor_online(base) || !motor_control_ready(base))
    {
        return;
    }
    switch (motor->config.mode)
    {
        case HT_MOTOR_MODE_TORQUE:
        {
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
            break;
        }
        case HT_MOTOR_MODE_POSITION_SPEED:
            motor->cmd_position = sign * base->target;
            motor->cmd_velocity = motor->config.speed_limit;
            break;
        case HT_MOTOR_MODE_MIT:
            if (base->control_mode == MOTOR_CONTROL_ANGLE)
            {
                motor->cmd_position = sign * base->target;
                motor->cmd_velocity = sign * motor_speed_feedforward(base);
                motor->cmd_kp = motor->config.kp;
            }
            else if (base->control_mode == MOTOR_CONTROL_SPEED)
            {
                motor->cmd_velocity = sign * (base->target + motor_speed_feedforward(base));
            }
            if (base->control_mode != MOTOR_CONTROL_OUTPUT)
            {
                motor->cmd_kd = motor->config.kd;
            }
            base->output = motor_output_feedforward(base);
            if (base->control_mode == MOTOR_CONTROL_OUTPUT)
            {
                base->output += base->target;
            }
            break;
    }
    motor->cmd_valid = true;
}

int32_t ht_motor_disable(ht_motor_t *motor)
{
    if (motor == NULL)
    {
        return ERR_INVAL;
    }
    const can_frame_t frame = { .id = motor->config.id, .len = 3, .data = { 0x01, 0x00, 0x00 } };

    motor_set_active(&motor->base, false);
    motor->cmd_valid = false;
    motor->braking = false;
    return bsp_can_send(motor->base.bus, &frame);
}

int32_t ht_motor_brake(ht_motor_t *motor)
{
    if (motor == NULL)
    {
        return ERR_INVAL;
    }
    const can_frame_t frame = { .id = motor->config.id, .len = 3, .data = { 0x01, 0x00, 0x0F } };

    motor_set_active(&motor->base, false);
    motor->cmd_valid = false;
    motor->braking = true;
    return bsp_can_send(motor->base.bus, &frame);
}

int32_t ht_motor_read_state(ht_motor_t *motor)
{
    const uint8_t data[] = { 0x17, 0x01 };

    if (motor == NULL)
    {
        return ERR_INVAL;
    }
    return bsp_can_send_extended(motor->base.bus, HT_REPLY_FLAG | motor->config.id, data, sizeof data);
}

int32_t ht_motor_send(ht_motor_t *motor)
{
    if (motor == NULL)
    {
        return ERR_INVAL;
    }
    motor_t *base = &motor->base;
    const ht_motor_config_t *config = &motor->config;
    if (!base->active || !motor_online(base) || base->hold_pending || !motor->cmd_valid ||
        !isfinite(base->output) || !isfinite(motor->cmd_position) || !isfinite(motor->cmd_velocity))
    {
        const can_frame_t stop =
        {
            .id = config->id,
            .len = 3,
            .data = { 0x01, 0x00, motor->braking && !base->active ? 0x0F : 0x00 },
        };
        int32_t ret = bsp_can_send(base->bus, &stop);
        if (ret == ERR_OK)
        {
            (void)ht_motor_read_state(motor);
        }
        return ret;
    }
    motor->braking = false;
    float output = base->reverse ? -base->output : base->output;
    float torque = clamp(output, -config->torque_limit, config->torque_limit) / config->torque_scale;
    can_frame_t frame = { .id = config->id };

    switch (config->mode)
    {
        case HT_MOTOR_MODE_TORQUE:
            frame.len = 4;
            frame.data[0] = 0x05;
            frame.data[1] = 0x13;
            write_i16(&frame.data[2], torque * HT_TORQUE_SCALE);
            break;
        case HT_MOTOR_MODE_POSITION_SPEED:
            frame.len = 8;
            frame.data[0] = 0x07;
            frame.data[1] = 0x35;
            write_i16(&frame.data[2], motor->cmd_velocity * (HT_VELOCITY_SCALE / HT_TWO_PI));
            write_i16(&frame.data[4], config->torque_limit / config->torque_scale * HT_TORQUE_SCALE);
            write_i16(&frame.data[6], motor->cmd_position * (HT_POSITION_SCALE / HT_TWO_PI));
            break;
        case HT_MOTOR_MODE_MIT:
        {
            uint16_t p = mit_field(motor->cmd_position / HT_TWO_PI, -3.2768f, 3.2767f, 16);
            uint16_t v = mit_field(motor->cmd_velocity / HT_TWO_PI, -2.0f, 2.0f, 12);
            uint16_t t = mit_field(torque, -10.0f, 10.0f, 12);
            uint16_t kp = mit_field(motor->cmd_kp * HT_TWO_PI / config->torque_scale, -400.0f, 400.0f, 12);
            uint16_t kd = mit_field(motor->cmd_kd * HT_TWO_PI / config->torque_scale, -100.0f, 100.0f, 12);
            const uint8_t data[] =
            {
                (uint8_t)p, (uint8_t)(p >> 8),
                (uint8_t)v, (uint8_t)((v >> 8) | ((t & 0x0Fu) << 4)), (uint8_t)(t >> 4),
                (uint8_t)kp, (uint8_t)((kp >> 8) | ((kd & 0x0Fu) << 4)), (uint8_t)(kd >> 4),
            };
            int32_t ret = bsp_can_send_extended(base->bus, HT_MIT_ID | config->id, data, sizeof data);
            if (ret == ERR_OK)
            {
                (void)ht_motor_read_state(motor);
            }
            return ret;
        }
        default:
            return ERR_INVAL;
    }
    int32_t ret = bsp_can_send(base->bus, &frame);
    if (ret == ERR_OK)
    {
        (void)ht_motor_read_state(motor);
    }
    return ret;
}

int32_t ht_motor_compute_bus(can_id_e bus)
{
    if ((unsigned)bus >= CAN_NUM)
    {
        return ERR_INVAL;
    }
    int32_t count = 0;
    for (uint8_t index = 0; ; index++)
    {
        motor_t *base = motor_registry_get(index);
        if (base == NULL)
        {
            return count;
        }
        if (base->bus == bus && is_ht_motor(base->type))
        {
            ht_motor_compute((ht_motor_t *)base);
            count++;
        }
    }
}

int32_t ht_motor_send_bus(can_id_e bus)
{
    if ((unsigned)bus >= CAN_NUM)
    {
        return ERR_INVAL;
    }
    int32_t count = 0;
    for (uint8_t index = 0; ; index++)
    {
        motor_t *base = motor_registry_get(index);
        if (base == NULL)
        {
            return count;
        }
        if (base->bus == bus && is_ht_motor(base->type) && ht_motor_send((ht_motor_t *)base) == ERR_OK)
        {
            count++;
        }
    }
}
