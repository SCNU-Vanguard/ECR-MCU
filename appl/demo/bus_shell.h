#ifndef APPL_DEMO_BUS_SHELL_H__
#define APPL_DEMO_BUS_SHELL_H__
/* 一条 CAN 上四台电机的开关、目标和 shell。三个任务各包含一次，用自己的电机表。 */
#include <string.h>
#include "component/common/err.h"
#include "cdd/motor/motor.h"
#include "cdd/motor/dm_motor/dm_motor.h"

typedef struct
{
    const char *name;          /* shell 里用的短名：3508 / 6020 / 3519 / 4310 */
    motor_t    *motor;
    dm_motor_t *dm;            /* DJI 为 NULL；DM 在 on/off 时还要发使能/失能帧 */
} demo_bus_motor_t;

static int32_t demo_bus_start(const demo_bus_motor_t *table, uint32_t count)
{
    int32_t ret = ERR_OK;

    for (uint32_t i = 0; i < count; i++)
    {
        motor_set_active(table[i].motor, true);
        if (table[i].dm != NULL)
        {
            int32_t enable_ret = dm_motor_enable(table[i].dm);

            if (ret == ERR_OK)
            {
                ret = enable_ret;
            }
        }
    }
    return ret;
}

static void demo_bus_set_active(const demo_bus_motor_t *item, bool active)
{
    motor_set_active(item->motor, active);
    if (item->dm == NULL)
    {
        return;
    }
    if (active)
    {
        (void)dm_motor_enable(item->dm);
    }
    else
    {
        (void)dm_motor_disable(item->dm);
    }
}

#ifdef CONFIG_COMPONENT_SHELL
#include "cdd/motor/motor_shell.h"
#include "component/shell/shell_cmd.h"

static const demo_bus_motor_t *demo_bus_find(const demo_bus_motor_t *table, uint32_t count, const char *name)
{
    for (uint32_t i = 0; i < count; i++)
    {
        if (strcmp(table[i].name, name) == 0)
        {
            return &table[i];
        }
    }
    return NULL;
}

static int demo_bus_shell_target(const char *command, const demo_bus_motor_t *table, uint32_t count,
                                 motor_control_mode_e mode, int argc, char *argv[])
{
    const demo_bus_motor_t *item;
    float value;

    if (argc != 3 || (item = demo_bus_find(table, count, argv[1])) == NULL || item->motor->control_mode != mode ||
        !motor_shell_parse_float(argv[2], &value))
    {
        shell_out("usage: %s [%s] [%s]\r\n", command, (mode == MOTOR_CONTROL_SPEED) ? "3508|3519" : "6020|4310",
                  (mode == MOTOR_CONTROL_SPEED) ? "rad/s" : "rad");
        return -1;
    }
    return motor_set_target(item->motor, value);
}

static int demo_bus_shell_active(const char *command, const demo_bus_motor_t *table, uint32_t count, bool active,
                                 int argc, char *argv[])
{
    if (argc == 1)
    {
        for (uint32_t i = 0; i < count; i++)
        {
            demo_bus_set_active(&table[i], active);
        }
        return 0;
    }
    const demo_bus_motor_t *item = (argc == 2) ? demo_bus_find(table, count, argv[1]) : NULL;

    if (item == NULL)
    {
        shell_out("usage: %s [3508|6020|3519|4310]\r\n", command);
        return -1;
    }
    demo_bus_set_active(item, active);
    return 0;
}

static int demo_bus_shell_pid(const char *command, const demo_bus_motor_t *table, uint32_t count, int argc, char *argv[])
{
    const demo_bus_motor_t *item;

    if ((argc != 2 && argc != 5) || (item = demo_bus_find(table, count, argv[1])) == NULL)
    {
        shell_out("usage: %s [3508|6020|3519|4310] [angle|speed] [kp|ki|kd|kf|out|int|ff|dead] [value]\r\n", command);
        return -1;
    }
    if (argc == 2)
    {
        char *args[] = { argv[0] };

        return motor_shell_pid(item->name, item->motor, 1, args);
    }
    char *args[] = { argv[0], argv[2], argv[3], argv[4] };

    return motor_shell_pid(item->name, item->motor, 4, args);
}

static int demo_bus_shell_info(const char *command, const demo_bus_motor_t *table, uint32_t count, int argc, char *argv[])
{
    if (argc == 1)
    {
        for (uint32_t i = 0; i < count; i++)
        {
            motor_shell_info(table[i].name, table[i].motor);
            if (table[i].dm != NULL)
            {
                shell_out("  dm_state = %u (1 = enabled)\r\n", (unsigned)table[i].dm->state);
            }
        }
        return 0;
    }
    const demo_bus_motor_t *item = (argc == 2) ? demo_bus_find(table, count, argv[1]) : NULL;

    if (item == NULL)
    {
        shell_out("usage: %s [3508|6020|3519|4310]\r\n", command);
        return -1;
    }
    motor_shell_info(item->name, item->motor);
    if (item->dm != NULL)
    {
        shell_out("  dm_state = %u (1 = enabled)\r\n", (unsigned)item->dm->state);
    }
    return 0;
}
#endif
#endif
