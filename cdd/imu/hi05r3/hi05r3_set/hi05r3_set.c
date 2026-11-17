/*
*   hi05r3_set.c
*   用于设置hi05r3 IMU模块的一些初始配置
*   使用之前请先使用上位机标定零偏、更改CAN发送比特率为1000KHz、更改模式为六轴模式
*   对照算法组下辖的导航和视觉需求，可能需要获取一些里程计、角速度、加速度、四元数需求
*   考虑总线上数据帧的精简，四元数会是一个更优的选择，至于电控需要的欧拉角就直接自己从四元数里获取
*/
#include "cdd/imu/hi05r3/hi05r3.h"
#include "component/common/err.h"
#include "component/common/types.h"
#include "ins_config.h"
#include "osal/osal.h"

#define HI05R3_PROFILE_ADDR              0x0006u
#define HI05R3_CAN_BITRATE_ADDR          0x009Au
#define HI05R3_CAN_NODE_ADDR             0x009Cu
#define HI05R3_CAN_OUTPUT_ENABLE_ADDR    0x009Du
#define HI05R3_ACCEL_PERIOD_ADDR         0x0134u
#define HI05R3_GYRO_PERIOD_ADDR          0x0137u
#define HI05R3_ROLL_PITCH_PERIOD_ADDR    0x013Du
#define HI05R3_YAW_PERIOD_ADDR           0x0141u
#define HI05R3_QUAT_PERIOD_ADDR          0x0146u
#define HI05R3_INCLI_PERIOD_ADDR         0x014Au

#define HI05R3_CAN_RATE_1MBPS            0u
#define HI05R3_CAN_RATE_500KBPS          2u
#define HI05R3_CAN_RATE_250KBPS          3u
#define HI05R3_CAN_RATE_125KBPS          4u
#define HI05R3_PROFILE_6_AXIS            0u
#define HI05R3_CAN_OUTPUT_DISABLED       0u
#define HI05R3_CAN_OUTPUT_ENABLED        1u
#define HI05R3_PERIOD_DISABLED_MS        0u

/* 默认输出周期和 ins_config.h 的输出频率保持一致。 */
_Static_assert(1000u % HI05R3_RATE_HZ == 0u, "HI05R3_RATE_HZ must divide 1000 so the output period is a whole number of ms");
#define HI05R3_DEFAULT_PERIOD_MS         (1000u / HI05R3_RATE_HZ)

static int32_t hi05r3_set_write_verified(uint16_t address, uint16_t value)
{
    uint16_t readback;
    int32_t ret;

    switch (address)
    {
    case HI05R3_CAN_BITRATE_ADDR:
        if (value != HI05R3_CAN_RATE_1MBPS && value != HI05R3_CAN_RATE_500KBPS &&
            value != HI05R3_CAN_RATE_250KBPS && value != HI05R3_CAN_RATE_125KBPS)
        {
            return ERR_INVAL;
        }
        break;
    case HI05R3_CAN_NODE_ADDR:
        if (value == 0u || value > 127u || value == HI05R3_HOST_ADDRESS)
        {
            return ERR_INVAL;
        }
        break;
    case HI05R3_PROFILE_ADDR:
        if (value != HI05R3_PROFILE_6_AXIS)
        {
            return ERR_INVAL;
        }
        break;
    case HI05R3_CAN_OUTPUT_ENABLE_ADDR:
        if (value != HI05R3_CAN_OUTPUT_DISABLED && value != HI05R3_CAN_OUTPUT_ENABLED)
        {
            return ERR_INVAL;
        }
        break;
    case HI05R3_ACCEL_PERIOD_ADDR:
    case HI05R3_GYRO_PERIOD_ADDR:
    case HI05R3_ROLL_PITCH_PERIOD_ADDR:
    case HI05R3_YAW_PERIOD_ADDR:
    case HI05R3_QUAT_PERIOD_ADDR:
    case HI05R3_INCLI_PERIOD_ADDR:
        if (value > 32767u)
        {
            return ERR_INVAL;
        }
        break;
    default:
        return ERR_INVAL;
    }
    ret = hi05r3_set_register(address, value);
    if (ret != ERR_OK)
    {
        return ret;
    }
    os_delay_ms(10u);
    ret = hi05r3_set_read_register(address, &readback);
    if (ret != ERR_OK)
    {
        return ret;
    }
    return readback == value ? ERR_OK : ERR_IO;
}

/* 写入本兵种的默认配置并逐项读回校验，最后保存并重启模块。 */
static int32_t hi05r3_set_apply_default(void)
{
    static const struct
    {
        uint16_t address;
        uint16_t value;
    } defaults[] = {
        { HI05R3_CAN_BITRATE_ADDR, HI05R3_CAN_RATE_1MBPS },
        { HI05R3_CAN_NODE_ADDR, HI05R3_NODE_ID },
        { HI05R3_PROFILE_ADDR, HI05R3_PROFILE_6_AXIS },
        { HI05R3_CAN_OUTPUT_ENABLE_ADDR, HI05R3_CAN_OUTPUT_ENABLED },
        { HI05R3_ACCEL_PERIOD_ADDR, HI05R3_DEFAULT_PERIOD_MS },
        { HI05R3_GYRO_PERIOD_ADDR, HI05R3_DEFAULT_PERIOD_MS },
        { HI05R3_ROLL_PITCH_PERIOD_ADDR, HI05R3_PERIOD_DISABLED_MS },
        { HI05R3_YAW_PERIOD_ADDR, HI05R3_PERIOD_DISABLED_MS },
        { HI05R3_QUAT_PERIOD_ADDR, HI05R3_DEFAULT_PERIOD_MS },
        { HI05R3_INCLI_PERIOD_ADDR, HI05R3_PERIOD_DISABLED_MS },
    };
    int32_t ret;

    for (size_t index = 0u; index < ARRAY_SIZE(defaults); index++)
    {
        ret = hi05r3_set_write_verified(defaults[index].address, defaults[index].value);
        if (ret != ERR_OK)
        {
            return ret;
        }
    }
    ret = hi05r3_set_register(HI05R3_CTRL_ADDR, HI05R3_CTRL_SAVE);
    if (ret != ERR_OK)
    {
        return ret;
    }
    os_delay_ms(10u);
    return hi05r3_set_register(HI05R3_CTRL_ADDR, HI05R3_CTRL_REBOOT);
}

#ifdef CONFIG_COMPONENT_SHELL
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include "component/shell/shell_cmd.h"

static bool hi05r3_set_parse_u16(const char *text, uint16_t *value)
{
    char *end;
    unsigned long parsed;

    if (text == NULL || *text == '\0' || value == NULL || *text == '-')
    {
        return false;
    }
    errno = 0;
    parsed = strtoul(text, &end, 0);
    if (errno != 0 || *end != '\0' || parsed > 65535u)
    {
        return false;
    }
    *value = (uint16_t)parsed;
    return true;
}

static int shell_hi05r3_set(int argc, char *argv[])
{
    static const struct
    {
        const char *name;
        uint16_t address;
    } commands[] = {
        { "rate", HI05R3_CAN_BITRATE_ADDR },
        { "node", HI05R3_CAN_NODE_ADDR },
        { "profile", HI05R3_PROFILE_ADDR },
        { "output", HI05R3_CAN_OUTPUT_ENABLE_ADDR },
        { "accel_ms", HI05R3_ACCEL_PERIOD_ADDR },
        { "gyro_ms", HI05R3_GYRO_PERIOD_ADDR },
        { "rpy_ms", HI05R3_ROLL_PITCH_PERIOD_ADDR },
        { "yaw_ms", HI05R3_YAW_PERIOD_ADDR },
        { "quat_ms", HI05R3_QUAT_PERIOD_ADDR },
        { "incli_ms", HI05R3_INCLI_PERIOD_ADDR },
    };
    uint16_t value;
    int32_t ret = ERR_INVAL;

    if (argc == 2 && strcmp(argv[1], "default") == 0)
    {
        ret = hi05r3_set_apply_default();
        if (ret == ERR_OK)
        {
            shell_out("hi05r3_set default: saved and reboot requested\r\n");
        }
        else
        {
            shell_out("hi05r3_set default failed: %ld\r\n", (long)ret);
        }
        return ret;
    }
    if (argc == 3 && strcmp(argv[1], "read") == 0 && hi05r3_set_parse_u16(argv[2], &value))
    {
        uint16_t readback;

        ret = hi05r3_set_read_register(value, &readback);
        if (ret == ERR_OK)
        {
            shell_out("hi05r3_set read 0x%04X: 0x%04X\r\n", (unsigned)value, (unsigned)readback);
        }
        else
        {
            shell_out("hi05r3_set read 0x%04X failed: %ld\r\n", (unsigned)value, (long)ret);
        }
        return ret;
    }
    if (argc == 2 && strcmp(argv[1], "save") == 0)
    {
        ret = hi05r3_set_register(HI05R3_CTRL_ADDR, HI05R3_CTRL_SAVE);
        shell_out("hi05r3_set save: %s\r\n", ret == ERR_OK ? "frame queued" : "CAN send failed");
        return ret;
    }
    if (argc == 2 && strcmp(argv[1], "reboot") == 0)
    {
        ret = hi05r3_set_register(HI05R3_CTRL_ADDR, HI05R3_CTRL_REBOOT);
        shell_out("hi05r3_set reboot: %s\r\n", ret == ERR_OK ? "frame queued" : "CAN send failed");
        return ret;
    }
    if (argc != 3 || !hi05r3_set_parse_u16(argv[2], &value))
    {
        shell_out("usage: hi05r3_set <read <addr>|rate|node|profile|output|accel_ms|gyro_ms|rpy_ms|yaw_ms|quat_ms|incli_ms|save|reboot|default> [value]\r\n");
        shell_out("rate codes: 0=1000k, 2=500k, 3=250k, 4=125k; periods are ms, 0 disables\r\n");
        return ERR_INVAL;
    }
    for (size_t index = 0u; index < ARRAY_SIZE(commands); index++)
    {
        if (strcmp(argv[1], commands[index].name) == 0)
        {
            ret = hi05r3_set_write_verified(commands[index].address, value);
            break;
        }
    }
    if (ret == ERR_OK)
    {
        shell_out("hi05r3_set %s: readback verified\r\n", argv[1]);
    }
    else
    {
        shell_out("hi05r3_set %s failed: %ld (invalid value / no response / readback mismatch)\r\n", argv[1], (long)ret);
    }
    return ret;
}
SHELL_CMD(hi05r3_set, shell_hi05r3_set, configure_hi05r3_over_j1939)
#endif
