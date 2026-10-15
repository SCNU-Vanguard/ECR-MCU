#include <string.h>
#include "cdd/motor/motor.h"
#include "bsp/tick/bsp_tick.h"
#include "component/common/err.h"
#include "osal/osal.h"

static motor_t *registered_motors[MOTOR_MAX_COUNT];
static uint8_t  registered_count;

int32_t motor_registry_add(motor_t *motor)
{
    if (motor == NULL || (unsigned)motor->bus >= CAN_NUM || motor->timeout_ms == 0u)
    {
        return ERR_INVAL;
    }
    if (registered_count >= MOTOR_MAX_COUNT)
    {
        return ERR_NOMEM;
    }
    for (uint8_t motor_index = 0; motor_index < registered_count; motor_index++)    /* 同总线、反馈 ID 相同 = 冲突 */
    {
        if (registered_motors[motor_index]->bus == motor->bus &&
            registered_motors[motor_index]->feedback_id == motor->feedback_id)
        {
            return ERR_BUSY;
        }
    }
    uint32_t irq_key = os_irq_save();

    registered_motors[registered_count] = motor;
    registered_count++;
    os_irq_restore(irq_key);
    return ERR_OK;
}

motor_t *motor_registry_get(uint8_t index)
{
    if (index >= registered_count)
    {
        return NULL;
    }
    return registered_motors[index];
}

bool motor_online(const motor_t *motor)
{
    return motor->got_first && (bsp_tick_ms() - motor->rx_ms) < motor->timeout_ms;
}

uint32_t motor_error(const motor_t *motor)
{
    uint32_t flags = motor->error_flags;

    if (!motor_online(motor))
    {
        flags |= MOTOR_ERROR_LOST;
    }
    return flags;
}

/* ---- 控制器 ---- */
void motor_control_setup(motor_t *motor, const motor_control_config_t *config)
{
    motor->has_control = (config != NULL);
    motor->active = false;
    motor->control_mode = MOTOR_CONTROL_TORQUE;
    motor->target = 0.0f;
    if (config == NULL)
    {
        return;
    }
    motor->options = config->options;
    pid_init(&motor->angle_pid, &config->angle_pid);
    pid_init(&motor->speed_pid, &config->speed_pid);
}

int32_t motor_set_target(motor_t *motor, motor_control_mode_e mode, float target)
{
    if (motor == NULL)
    {
        return ERR_INVAL;
    }
    motor->control_mode = mode;
    motor->target = target;
    return ERR_OK;
}

void motor_pids_clear(motor_t *motor)
{
    pid_clear(&motor->angle_pid);
    pid_clear(&motor->speed_pid);
}

void motor_set_active(motor_t *motor, bool active)
{
    if (!active)
    {
        motor_pids_clear(motor);
    }
    motor->hold_pending = active;
    motor->active = active;
}

bool motor_control_ready(motor_t *motor)
{
    if (!motor->got_first)
    {
        return false;
    }
    if (motor->hold_pending)
    {
        motor->target = (motor->control_mode == MOTOR_CONTROL_ANGLE) ? motor->single_angle_rad : 0.0f;
        motor_pids_clear(motor);
        motor->hold_pending = false;
    }
    return true;
}

float motor_angle_loop(motor_t *motor, float angle_target)
{
    float angle_measure;

    if (motor->options.angle_feedback != NULL)
    {
        angle_measure = *motor->options.angle_feedback;
    }
    else
    {
        angle_measure = motor->single_angle_rad;
    }
    return pid_angle_compute(&motor->angle_pid, angle_measure, angle_target);
}

float motor_speed_feedforward(const motor_t *motor)
{
    return (motor->options.speed_feedforward != NULL) ? *motor->options.speed_feedforward : 0.0f;
}

float motor_torque_feedforward(const motor_t *motor)
{
    return (motor->options.torque_feedforward != NULL) ? *motor->options.torque_feedforward : 0.0f;
}

float motor_speed_loop(motor_t *motor, float speed_reference)
{
    float speed_measure = (motor->options.speed_feedback != NULL) ? *motor->options.speed_feedback : motor->speed_rad_s;

    speed_reference += motor_speed_feedforward(motor);
    return pid_increment_compute(&motor->speed_pid, speed_measure, speed_reference);
}
