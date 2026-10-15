#include "cdd/motor/motor_shell.h"

#ifdef CONFIG_COMPONENT_SHELL
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "component/shell/shell_cmd.h"

static void format_milli(char *buffer, uint32_t buffer_size, float value)
{
    const char *sign = "";

    if (value < 0.0f)
    {
        sign = "-";
        value = -value;
    }
    uint32_t milli = (uint32_t)(value * 1000.0f + 0.5f);

    snprintf(buffer, buffer_size, "%s%lu.%03lu", sign, (unsigned long)(milli / 1000u), (unsigned long)(milli % 1000u));
}

static void print_milli(const char *label, float value)
{
    char text[20];

    format_milli(text, sizeof text, value);
    shell_out("  %s = %s\r\n", label, text);
}

bool motor_shell_parse_float(const char *text, float *value)
{
    char *end = NULL;
    float parsed = strtof(text, &end);

    if (end == text || *end != '\0')
    {
        return false;
    }
    *value = parsed;
    return true;
}

void motor_shell_info(const char *name, const motor_t *motor)
{
    static const char *const mode_names[] = { "torque", "speed", "angle" };

    shell_out("%s: online=%d error=0x%lx active=%d mode=%s\r\n", name, (int)motor_online(motor),
              (unsigned long)motor_error(motor), (int)motor->active, mode_names[motor->control_mode]);
    print_milli("target", motor->target);
    print_milli("single_angle_rad", motor->single_angle_rad);
    print_milli("total_angle_rad", motor->total_angle_rad);
    print_milli("speed_rad_s", motor->speed_rad_s);
    print_milli("torque", motor->torque);
    print_milli("output", motor->output);
    print_milli("temperature", motor->temperature);
}

static void print_pid(const char *loop_name, const pid_config_t *config)
{
    shell_out("%s:\r\n", loop_name);
    print_milli("kp", config->kp);
    print_milli("ki", config->ki);
    print_milli("kd", config->kd);
    print_milli("kf", config->kf);
    print_milli("out (output_limit)", config->output_limit);
    print_milli("int (integral_limit)", config->integral_limit);
    print_milli("ff (feedforward_limit)", config->feedforward_limit);
    print_milli("dead (dead_band)", config->dead_band);
}

static float *find_pid_field(pid_config_t *config, const char *field)
{
    if (strcmp(field, "kp") == 0)
    {
        return &config->kp;
    }
    if (strcmp(field, "ki") == 0)
    {
        return &config->ki;
    }
    if (strcmp(field, "kd") == 0)
    {
        return &config->kd;
    }
    if (strcmp(field, "kf") == 0)
    {
        return &config->kf;
    }
    if (strcmp(field, "out") == 0)
    {
        return &config->output_limit;
    }
    if (strcmp(field, "int") == 0)
    {
        return &config->integral_limit;
    }
    if (strcmp(field, "ff") == 0)
    {
        return &config->feedforward_limit;
    }
    if (strcmp(field, "dead") == 0)
    {
        return &config->dead_band;
    }
    return NULL;
}

int motor_shell_pid(const char *name, motor_t *motor, int argc, char *argv[])
{
    if (argc == 1)
    {
        shell_out("%s pid:\r\n", name);
        print_pid("angle", &motor->angle_pid.config);
        print_pid("speed", &motor->speed_pid.config);
        return 0;
    }
    if (argc != 4)
    {
        shell_out("usage: %s [angle|speed] [kp|ki|kd|kf|out|int|ff|dead] [value]\r\n", argv[0]);
        return -1;
    }
    pid_controller_t *controller = NULL;

    if (strcmp(argv[1], "angle") == 0)
    {
        controller = &motor->angle_pid;
    }
    else if (strcmp(argv[1], "speed") == 0)
    {
        controller = &motor->speed_pid;
    }
    float *field = (controller != NULL) ? find_pid_field(&controller->config, argv[2]) : NULL;
    float value;

    if (field == NULL || !motor_shell_parse_float(argv[3], &value))
    {
        shell_out("usage: %s [angle|speed] [kp|ki|kd|kf|out|int|ff|dead] [value]\r\n", argv[0]);
        return -1;
    }
    *field = value;
    print_milli(argv[2], *field);
    return 0;
}
#endif
