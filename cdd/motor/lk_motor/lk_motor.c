#include <string.h>
#include "cdd/motor/lk_motor/lk_motor.h"
#include "bsp/can/bsp_can.h"
#include "bsp/tick/bsp_tick.h"
#include "component/common/err.h"
#include "osal/osal.h"
#include "osal/event.h"

/* 命令字节。0xA1 的回复和 0x9C 布局相同，只是 DATA[0] 不同。 */
#define LK_CMD_STOP           0x81u
#define LK_CMD_RUN            0x88u
#define LK_CMD_CLEAR_ERROR    0x9Bu
#define LK_CMD_READ_STATE1    0x9Au
#define LK_CMD_READ_STATE2    0x9Cu
#define LK_CMD_TORQUE         0xA1u

#define LK_ID_BASE            0x140u
#define LK_ID_MIN             1u
#define LK_ID_MAX             32u
#define LK_IQ_MAX             2048
#define ENCODER_COUNTS        65536
#define ENCODER_HALF          32768
#define PI                    3.1415927f
#define TWO_PI                6.2831853f
#define DPS_TO_RAD_PER_SEC    0.017453292f

static bool subscribed[CAN_NUM];

static uint16_t can_id_of(uint8_t id)
{
    return (uint16_t)(LK_ID_BASE + id);
}

static int16_t clamp_iq(int32_t iq)
{
    if (iq > LK_IQ_MAX)
    {
        return LK_IQ_MAX;
    }
    if (iq < -LK_IQ_MAX)
    {
        return (int16_t)-LK_IQ_MAX;
    }
    return (int16_t)iq;
}

static int32_t send_command(lk_motor_t *motor, uint8_t command)
{
    can_frame_t frame;

    frame.id = can_id_of(motor->id);
    frame.len = 8;
    memset(frame.data, 0, sizeof frame.data);
    frame.data[0] = command;
    return bsp_can_send(motor->base.bus, &frame);
}

static void update_state2(lk_motor_t *motor, const uint8_t *data)
{
    int16_t speed_dps = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
    uint16_t encoder = (uint16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
    int32_t encoder_delta;

    motor->current = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
    motor->speed_dps = speed_dps;
    motor->last_encoder = motor->encoder;
    motor->encoder = encoder;

    if (!motor->base.got_first)
    {
        motor->base.got_first = true;
        motor->last_encoder = encoder;
    }
    else
    {
        encoder_delta = (int32_t)encoder - (int32_t)motor->last_encoder;
        if (encoder_delta > ENCODER_HALF)
        {
            motor->round_count--;
        }
        else if (encoder_delta < -ENCODER_HALF)
        {
            motor->round_count++;
        }
    }

    float sign = motor->base.reverse ? -1.0f : 1.0f;
    float single = (float)encoder * TWO_PI / (float)ENCODER_COUNTS;

    if (single >= PI)
    {
        single -= TWO_PI;
    }
    motor->base.single_angle_rad = sign * single;
    motor->base.total_angle_rad = sign * ((float)encoder / (float)ENCODER_COUNTS + (float)motor->round_count) * TWO_PI;
    motor->base.speed_rad_s = sign * (float)speed_dps * DPS_TO_RAD_PER_SEC;
    motor->base.torque = sign * (float)motor->current;
    motor->base.temperature = (float)(int8_t)data[1];
    motor->base.rx_ms = bsp_tick_ms();
}

static void update_error(lk_motor_t *motor, uint8_t error_state)
{
    if (error_state != 0u)
    {
        motor->base.error_flags |= MOTOR_ERROR_FAULT;
    }
    else
    {
        motor->base.error_flags &= ~MOTOR_ERROR_FAULT;
    }
    motor->base.rx_ms = bsp_tick_ms();
}

static void lk_motor_run_event(void *argument, const uint8_t *message, uint16_t len)
{
    can_event_t event;

    (void)argument;
    if (message == NULL || len != sizeof event)
    {
        return;
    }
    memcpy(&event, message, sizeof event);
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
        lk_motor_t *motor = (lk_motor_t *)base;
        uint32_t irq_key = os_irq_save();

        if (event.data[0] == LK_CMD_TORQUE || event.data[0] == LK_CMD_READ_STATE2)
        {
            update_state2(motor, event.data);
        }
        else if (event.data[0] == LK_CMD_READ_STATE1 || event.data[0] == LK_CMD_CLEAR_ERROR)
        {
            update_error(motor, event.data[7]);
        }
        os_irq_restore(irq_key);
        return;
    }
}
EVENT_TASK_DEFINE(lk_motor_task, OS_PRIORITY_HIGHEST, OS_STACK_MEDIUM, EVENT_QUEUE, sizeof(can_event_t), OS_QUEUE_LEN_32, OS_TIMEOUT_NONE, lk_motor_run_event, NULL);

int32_t lk_motor_register(lk_motor_t *motor, const lk_motor_config_t *config)
{
    if (motor == NULL || config == NULL || config->id < LK_ID_MIN || config->id > LK_ID_MAX || config->timeout_ms == 0u)
    {
        return ERR_INVAL;
    }
    memset(motor, 0, sizeof *motor);
    motor->base.bus = config->bus;
    motor->base.type = MOTOR_TYPE_MF9025;
    motor->base.feedback_id = can_id_of(config->id);
    motor->base.reverse = config->reverse;
    motor->base.timeout_ms = config->timeout_ms;
    motor->id = config->id;
    motor_control_setup(&motor->base, config->control);

    int32_t add_ret = motor_registry_add(&motor->base);

    if (add_ret != ERR_OK)
    {
        return add_ret;
    }
    if (!subscribed[config->bus])
    {
        int32_t subscribe_ret = bsp_can_subscribe_event(config->bus, can_id_of(LK_ID_MIN), can_id_of(LK_ID_MAX), &lk_motor_task);

        if (subscribe_ret != ERR_OK)
        {
            return subscribe_ret;
        }
        subscribed[config->bus] = true;
    }
    (void)lk_motor_run(motor);
    return ERR_OK;
}

void lk_motor_set_iq(lk_motor_t *motor, int16_t iq)
{
    if (!motor->base.has_control)
    {
        motor->iq = clamp_iq(iq);
    }
}

int32_t lk_motor_run(lk_motor_t *motor)
{
    return send_command(motor, LK_CMD_RUN);
}

int32_t lk_motor_stop(lk_motor_t *motor)
{
    return send_command(motor, LK_CMD_STOP);
}

int32_t lk_motor_clear_error(lk_motor_t *motor)
{
    return send_command(motor, LK_CMD_CLEAR_ERROR);
}

void lk_motor_zero(lk_motor_t *motor)
{
    uint32_t irq_key = os_irq_save();

    motor->round_count = 0;
    motor->base.total_angle_rad = 0.0f;
    motor->base.got_first = false;
    os_irq_restore(irq_key);
}

void lk_motor_compute(lk_motor_t *motor)
{
    motor_t *base = &motor->base;

    if (!base->has_control)
    {
        return;
    }
    if (!base->active || !motor_control_ready(base))
    {
        base->output = 0.0f;
        motor->iq = 0;
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
        if (base->type != MOTOR_TYPE_MF9025 || base->bus != bus || !base->has_control)
        {
            continue;
        }
        lk_motor_compute((lk_motor_t *)base);
        computed_count++;
    }
    return computed_count;
}

int32_t lk_motor_send(lk_motor_t *motor)
{
    can_frame_t frame;
    int16_t iq = motor->iq;

    if (motor->base.has_control)
    {
        if (motor->base.active)
        {
            float physical = motor->base.reverse ? -motor->base.output : motor->base.output;

            iq = clamp_iq((int32_t)physical);
        }
        else
        {
            iq = 0;
        }
        motor->iq = iq;
    }
    frame.id = can_id_of(motor->id);
    frame.len = 8;
    memset(frame.data, 0, sizeof frame.data);
    frame.data[0] = LK_CMD_TORQUE;
    frame.data[4] = (uint8_t)iq;
    frame.data[5] = (uint8_t)((uint16_t)iq >> 8);
    return bsp_can_send(motor->base.bus, &frame);
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
