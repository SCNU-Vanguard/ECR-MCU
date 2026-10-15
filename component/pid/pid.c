#include <string.h>
#include "component/pid/pid.h"

#define PID_PI      3.1415927f
#define PID_TWO_PI  6.2831853f

static float limit_value(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static float absolute_value(float value)
{
    return (value < 0.0f) ? -value : value;
}

float pid_wrap_pi(float angle)
{
    float shifted = angle + PID_PI;
    int32_t turns = (int32_t)(shifted / PID_TWO_PI);

    shifted -= (float)turns * PID_TWO_PI;
    if (shifted < 0.0f)
    {
        shifted += PID_TWO_PI;
    }
    return shifted - PID_PI;
}

void pid_init(pid_controller_t *pid, const pid_config_t *config)
{
    memset(pid, 0, sizeof *pid);
    pid->config = *config;
}

void pid_clear(pid_controller_t *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->previous_error = 0.0f;
    pid->last_target = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float pid_angle_compute(pid_controller_t *pid, float measure, float target)
{
    target = pid_wrap_pi(target);
    pid->error = pid_wrap_pi(target - measure);

    if (absolute_value(pid->error) > pid->config.dead_band)
    {
        float proportional = pid->config.kp * pid->error;
        float derivative = pid->config.kd * (pid->error - pid->last_error);

        pid->integral += pid->config.ki * pid->error;
        pid->integral = limit_value(pid->integral, pid->config.integral_limit);
        pid->output = limit_value(proportional + pid->integral + derivative, pid->config.output_limit);
    }
    else                                                    /* 进入死区：输出清零，积分保留 */
    {
        pid->output = 0.0f;
    }
    pid->last_target = target;
    pid->last_error = pid->error;
    return pid->output;
}

float pid_position_compute(pid_controller_t *pid, float measure, float target)
{
    pid->error = target - measure;

    if (absolute_value(pid->error) > pid->config.dead_band)
    {
        float proportional = pid->config.kp * pid->error;
        float derivative = pid->config.kd * (pid->error - pid->last_error);

        pid->integral += pid->config.ki * pid->error;
        pid->integral = limit_value(pid->integral, pid->config.integral_limit);
        pid->output = limit_value(proportional + pid->integral + derivative, pid->config.output_limit);
    }
    else                                                    /* 进入死区：输出清零，积分保留 */
    {
        pid->output = 0.0f;
    }
    pid->last_target = target;
    pid->last_error = pid->error;
    return pid->output;
}

float pid_increment_compute(pid_controller_t *pid, float measure, float target)
{
    pid->error = target - measure;

    if (absolute_value(pid->error) > pid->config.dead_band)
    {
        float proportional = pid->config.kp * (pid->error - pid->last_error);
        float integral = limit_value(pid->config.ki * pid->error, pid->config.integral_limit);
        float derivative = pid->config.kd * (pid->error - 2.0f * pid->last_error + pid->previous_error);
        float feedforward = limit_value(pid->config.kf * (target - pid->last_target),
                                        pid->config.feedforward_limit);

        pid->output += proportional + integral + derivative + feedforward;
        pid->output = limit_value(pid->output, pid->config.output_limit);
    }
    else                                                    /* 进入死区：输出清零 */
    {
        pid->output = 0.0f;
    }
    pid->last_target = target;
    pid->previous_error = pid->last_error;
    pid->last_error = pid->error;
    return pid->output;
}
