/* 电机 PID 自动整定模块（复用 UART_LOG / letter-shell 单串口）：
 *   下行：letter-shell 命令（pid list / select / speed / pos / mode / target / vlim / log / stop / release / status）
 *   上行：周期控制任务计算后通过 UART_LOG 回传带前缀文本行：
 *         PID:<target_vel>,<actual_vel>,<target_pos>,<actual_pos>,<output>\r\n
 *
 * 说明：
 *   1. 使用轻量整数转小数格式化，不依赖 -u _printf_float，兼容 nano.specs 且耗时仅约数微秒。
 *   2. 默认上电不开启上报、不接管。pid select 起开始接管：该电机不再跟应用任务，保持到
 *      pid mode speed/pos 才按整定目标出力。pid release 交回原控制模式，并由应用任务重新打开输出。 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "bsp/include/bsp_ids.h"
#include "bsp/uart/bsp_uart.h"
#include "cdd/motor/motor.h"
#include "cdd/motor/motor_pid_tune.h"
#include "component/shell/shell_cmd.h"

static motor_t             *tune_motor;
static uint8_t              tune_motor_index;
static bool                 tune_override;
static bool                 tune_active;
static motor_control_mode_e tune_mode = MOTOR_CONTROL_SPEED;
static float                tune_target;
static float                tune_vlim_min = -1000.0f;
static float                tune_vlim_max =  1000.0f;

static bool                 tune_log_enabled;
static uint32_t             tune_log_div = 1u;
static uint32_t             tune_log_tick;
static motor_control_mode_e tune_saved_mode;
static bool                 tune_mode_saved;

/* 记下应用任务原来的控制模式。重复进入不覆盖，避免把已经改成 speed/pos 的模式当成原模式。 */
static void tune_begin(motor_t *motor)
{
    if (!tune_mode_saved)
    {
        tune_saved_mode = motor->control_mode;
        tune_mode_saved = true;
    }
    tune_override = true;
}

/* 交回应用任务：恢复模式并关掉输出。应用任务看到 active 为假后会自己重新打开。 */
static void tune_hand_back(motor_t *motor)
{
    tune_override = false;
    tune_active = false;
    if (motor != NULL && tune_mode_saved)
    {
        motor->control_mode = tune_saved_mode;
        tune_mode_saved = false;
    }
}

/* 停在接管里：输出关掉，应用任务写的目标不会生效。 */
static void tune_stop(motor_t *motor)
{
    motor_pids_clear(motor);
    tune_target = 0.0f;
    tune_active = false;
    tune_begin(motor);
    motor_set_active(motor, false);
}

static const char *motor_type_name(motor_type_e type)
{
    switch (type)
    {
        case MOTOR_TYPE_M3508:  return "M3508";
        case MOTOR_TYPE_M2006:  return "M2006";
        case MOTOR_TYPE_GM6020: return "GM6020";
        case MOTOR_TYPE_J4310:  return "J4310";
        case MOTOR_TYPE_J8009P: return "J8009P";
        case MOTOR_TYPE_S3519:  return "S3519";
        case MOTOR_TYPE_MF9025: return "MF9025";
        default:                return "UNKNOWN";
    }
}

static const char *control_mode_name(motor_control_mode_e mode)
{
    switch (mode)
    {
        case MOTOR_CONTROL_SPEED:  return "speed";
        case MOTOR_CONTROL_ANGLE:  return "pos";
        default:                   return "output";
    }
}

/* 零堆分配、兼容 nano.specs 的定点数转 ASCII（避免 printf %f 失效或耗时） */
static size_t append_char(char *buf, size_t cap, size_t pos, char ch)
{
    if (pos + 1u < cap)
    {
        buf[pos++] = ch;
    }
    return pos;
}

static size_t append_str(char *buf, size_t cap, size_t pos, const char *s)
{
    while (*s != '\0' && pos + 1u < cap)
    {
        buf[pos++] = *s++;
    }
    return pos;
}

static size_t append_uint(char *buf, size_t cap, size_t pos, uint32_t val)
{
    char tmp[12];
    size_t len = 0u;

    do
    {
        tmp[len++] = (char)('0' + (val % 10u));
        val /= 10u;
    } while (val > 0u && len < sizeof tmp);

    while (len > 0u)
    {
        pos = append_char(buf, cap, pos, tmp[--len]);
    }
    return pos;
}

static size_t append_float(char *buf, size_t cap, size_t pos, float val, uint8_t decimals)
{
    static const uint32_t pow10_table[5] = { 1u, 10u, 100u, 1000u, 10000u };

    if (!isfinite(val))
    {
        return append_str(buf, cap, pos, "0");
    }
    if (decimals > 4u)
    {
        decimals = 4u;
    }
    if (val < 0.0f)
    {
        pos = append_char(buf, cap, pos, '-');
        val = -val;
    }
    if (val > 200000.0f)
    {
        val = 200000.0f;
    }

    uint32_t scale = pow10_table[decimals];
    uint64_t scaled = (uint64_t)(val * (float)scale + 0.5f);
    uint32_t int_part = (uint32_t)(scaled / scale);
    uint32_t frac_part = (uint32_t)(scaled % scale);

    pos = append_uint(buf, cap, pos, int_part);
    if (decimals > 0u)
    {
        pos = append_char(buf, cap, pos, '.');
        uint32_t div = scale / 10u;
        while (div > 0u)
        {
            pos = append_char(buf, cap, pos, (char)('0' + ((frac_part / div) % 10u)));
            div /= 10u;
        }
    }
    return pos;
}

static void format_float_str(char *buf, size_t cap, float val, uint8_t decimals)
{
    size_t pos = append_float(buf, cap, 0u, val, decimals);
    buf[pos] = '\0';
}

/* 确保已有选中的电机；若尚未显式 select，则默认选中 index=0 */
static motor_t *ensure_tune_motor(void)
{
    if (tune_motor == NULL)
    {
        tune_motor = motor_registry_get(0u);
        tune_motor_index = 0u;
    }
    return tune_motor;
}

void motor_pid_tune_pre_compute(can_id_e bus)
{
    motor_t *motor = tune_motor;

    if (!tune_override || motor == NULL || motor->bus != bus)
    {
        return;
    }
    /* 只选中、还没进 speed/pos：每拍压过应用任务刚打开的输出，电机保持，不跟遥控器。 */
    if (!tune_active)
    {
        motor->active = false;
        return;
    }
    motor->control_mode = tune_mode;
    motor->active = true;
    motor->hold_pending = false;
    motor->target = tune_target;
}

void motor_pid_tune_post_compute(can_id_e bus)
{
    motor_t *motor = tune_motor;

    if (!tune_log_enabled || motor == NULL || motor->bus != bus)
    {
        return;
    }
    tune_log_tick++;
    if (tune_log_div > 1u && (tune_log_tick % tune_log_div) != 0u)
    {
        return;
    }

    float actual_vel = *motor->signals.speed_feedback;
    float actual_pos = *motor->signals.angle_feedback;
    float target_vel = (motor->control_mode == MOTOR_CONTROL_SPEED)
                           ? motor->target
                           : motor->angle_pid.output;
    float target_pos = (motor->control_mode == MOTOR_CONTROL_ANGLE)
                           ? motor->target
                           : actual_pos;
    float output = motor->output;

    char line[96];
    size_t pos = 0u;
    pos = append_str(line, sizeof line, pos, "PID:");
    pos = append_float(line, sizeof line, pos, target_vel, 3u);
    pos = append_char(line, sizeof line, pos, ',');
    pos = append_float(line, sizeof line, pos, actual_vel, 3u);
    pos = append_char(line, sizeof line, pos, ',');
    pos = append_float(line, sizeof line, pos, target_pos, 4u);
    pos = append_char(line, sizeof line, pos, ',');
    pos = append_float(line, sizeof line, pos, actual_pos, 4u);
    pos = append_char(line, sizeof line, pos, ',');
    pos = append_float(line, sizeof line, pos, output, 2u);
    pos = append_str(line, sizeof line, pos, "\r\n");

    (void)bsp_uart_write(UART_LOG, (const uint8_t *)line, (uint32_t)pos);
}

static void print_motor_status(const motor_t *motor, uint8_t idx)
{
    char skp[20], ski[20], pkp[20], pkd[20];

    format_float_str(skp, sizeof skp, motor->speed_pid.config.kp, 4u);
    format_float_str(ski, sizeof ski, motor->speed_pid.config.ki, 4u);
    format_float_str(pkp, sizeof pkp, motor->angle_pid.config.kp, 4u);
    format_float_str(pkd, sizeof pkd, motor->angle_pid.config.kd, 4u);

    shell_out("pid[%u] %s bus=%d id=0x%x online=%d override=%d active=%d mode=%s log=%s(div=%u)\r\n",
              (unsigned)idx,
              motor_type_name(motor->type),
              (int)motor->bus,
              (unsigned)motor->feedback_id,
              motor_online(motor) ? 1 : 0,
              tune_override ? 1 : 0,
              tune_active ? 1 : 0,
              control_mode_name(motor->control_mode),
              tune_log_enabled ? "on" : "off",
              (unsigned)tune_log_div);
    shell_out("  speed PI: kp=%s ki=%s | pos PD: kp=%s kd=%s\r\n", skp, ski, pkp, pkd);
}

static int shell_pid(int argc, char *argv[])
{
    if (argc < 2)
    {
        shell_out("usage: pid <list|select|speed|pos|mode|target|vlim|log|stop|release|status>\r\n");
        return 0;
    }

    const char *sub = argv[1];

    /* 1) pid list —— 列出全部已注册电机 */
    if (strcmp(sub, "list") == 0)
    {
        for (uint8_t idx = 0u; idx < MOTOR_MAX_COUNT; idx++)
        {
            motor_t *m = motor_registry_get(idx);
            if (m == NULL)
            {
                if (idx == 0u)
                {
                    shell_out("pid: no motor registered\r\n");
                }
                break;
            }
            shell_out("  [%u] %s bus=%d fb=0x%x mode=%s online=%d%s\r\n",
                      (unsigned)idx,
                      motor_type_name(m->type),
                      (int)m->bus,
                      (unsigned)m->feedback_id,
                      control_mode_name(m->control_mode),
                      motor_online(m) ? 1 : 0,
                      (m == tune_motor) ? " *" : "");
        }
        return 0;
    }

    /* 2) pid select <index> —— 选择待整定电机 */
    if (strcmp(sub, "select") == 0 && argc >= 3)
    {
        long idx = strtol(argv[2], NULL, 10);
        motor_t *m = (idx >= 0 && idx < MOTOR_MAX_COUNT) ? motor_registry_get((uint8_t)idx) : NULL;
        if (m == NULL)
        {
            shell_out("pid: invalid motor index %ld\r\n", idx);
            return -1;
        }
        if (tune_override && tune_motor != NULL && tune_motor != m)
        {
            tune_hand_back(tune_motor);
        }
        tune_motor = m;
        tune_motor_index = (uint8_t)idx;
        tune_active = false;
        tune_begin(m);
        motor_set_active(m, false);
        print_motor_status(m, tune_motor_index);
        return 0;
    }

    motor_t *motor = ensure_tune_motor();
    if (motor == NULL)
    {
        shell_out("pid: no motor registered\r\n");
        return -1;
    }

    /* 3) pid status */
    if (strcmp(sub, "status") == 0)
    {
        print_motor_status(motor, tune_motor_index);
        return 0;
    }

    /* 4) pid speed <kp> <ki> —— 速度环 PI */
    if (strcmp(sub, "speed") == 0 && argc >= 4)
    {
        float kp = strtof(argv[2], NULL);
        float ki = strtof(argv[3], NULL);
        char s1[20], s2[20];

        motor->speed_pid.config.kp = kp;
        motor->speed_pid.config.ki = ki;
        motor->speed_pid.config.kd = 0.0f;
        motor->speed_pid.config.kf = 0.0f;
        format_float_str(s1, sizeof s1, kp, 4u);
        format_float_str(s2, sizeof s2, ki, 4u);
        shell_out("ok speed kp=%s ki=%s\r\n", s1, s2);
        return 0;
    }

    /* 5) pid pos <kp> <kd> —— 位置环 PD */
    if (strcmp(sub, "pos") == 0 && argc >= 4)
    {
        float kp = strtof(argv[2], NULL);
        float kd = strtof(argv[3], NULL);
        char s1[20], s2[20];

        motor->angle_pid.config.kp = kp;
        motor->angle_pid.config.ki = 0.0f;
        motor->angle_pid.config.kd = kd;
        motor->angle_pid.config.kf = 0.0f;
        format_float_str(s1, sizeof s1, kp, 4u);
        format_float_str(s2, sizeof s2, kd, 4u);
        shell_out("ok pos kp=%s kd=%s\r\n", s1, s2);
        return 0;
    }

    /* 6) pid mode <speed|pos|stop> */
    if (strcmp(sub, "mode") == 0 && argc >= 3)
    {
        if (strcmp(argv[2], "speed") == 0)
        {
            motor_pids_clear(motor);
            tune_mode = MOTOR_CONTROL_SPEED;
            tune_target = 0.0f;
            tune_active = true;
            tune_begin(motor);
            shell_out("ok mode speed\r\n");
            return 0;
        }
        if (strcmp(argv[2], "pos") == 0)
        {
            motor_pids_clear(motor);
            tune_mode = MOTOR_CONTROL_ANGLE;
            tune_target = *motor->signals.angle_feedback;
            tune_active = true;
            tune_begin(motor);
            shell_out("ok mode pos\r\n");
            return 0;
        }
        if (strcmp(argv[2], "stop") == 0)
        {
            tune_stop(motor);
            shell_out("ok mode stop\r\n");
            return 0;
        }
    }

    /* 7) pid target <val> */
    if (strcmp(sub, "target") == 0 && argc >= 3)
    {
        float val = strtof(argv[2], NULL);
        char s[20];

        if (tune_mode == MOTOR_CONTROL_SPEED)
        {
            if (val > tune_vlim_max) val = tune_vlim_max;
            if (val < tune_vlim_min) val = tune_vlim_min;
        }
        tune_target = val;
        if (!tune_override)
        {
            motor_set_target(motor, val);
        }
        format_float_str(s, sizeof s, val, 4u);
        shell_out("ok target=%s\r\n", s);
        return 0;
    }

    /* 8) pid vlim <vmin> <vmax> */
    if (strcmp(sub, "vlim") == 0 && argc >= 4)
    {
        float vmin = strtof(argv[2], NULL);
        float vmax = strtof(argv[3], NULL);
        float sym = (vmax > -vmin) ? vmax : -vmin;
        char s1[20], s2[20];

        tune_vlim_min = vmin;
        tune_vlim_max = vmax;
        if (sym > 0.0f)
        {
            motor->angle_pid.config.output_limit = sym;
        }
        format_float_str(s1, sizeof s1, vmin, 3u);
        format_float_str(s2, sizeof s2, vmax, 3u);
        shell_out("ok vlim=[%s, %s]\r\n", s1, s2);
        return 0;
    }

    /* 9) pid log <on|off|div> —— 复用 UART_LOG 回传 PID: 文本数据 */
    if (strcmp(sub, "log") == 0 && argc >= 3)
    {
        if (strcmp(argv[2], "on") == 0)
        {
            tune_log_div = 1u;
            tune_log_tick = 0u;
            tune_log_enabled = true;
            shell_out("ok log on (div=1)\r\n");
            return 0;
        }
        if (strcmp(argv[2], "off") == 0)
        {
            tune_log_enabled = false;
            shell_out("ok log off\r\n");
            return 0;
        }
        long div = strtol(argv[2], NULL, 10);
        if (div >= 1 && div <= 1000)
        {
            tune_log_div = (uint32_t)div;
            tune_log_tick = 0u;
            tune_log_enabled = true;
            shell_out("ok log on (div=%ld)\r\n", div);
            return 0;
        }
    }

    /* 10) pid stop —— 急停（保持接管防止原任务重启） */
    if (strcmp(sub, "stop") == 0)
    {
        tune_stop(motor);
        shell_out("ok stop\r\n");
        return 0;
    }

    /* 11) pid release —— 退出整定接管，恢复正常机器人任务控制 */
    if (strcmp(sub, "release") == 0)
    {
        tune_log_enabled = false;
        tune_hand_back(motor);
        shell_out("ok release\r\n");
        return 0;
    }

    shell_out("pid: invalid args\r\n");
    return -1;
}
SHELL_CMD(pid, shell_pid, pid_auto_tuner_shell_and_log_telemetry)
