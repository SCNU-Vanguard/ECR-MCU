/* 上位机：100 Hz。打开的协议在这一次任务里发出去。
 * 自瞄目标、目标移速只收不发，云台和底盘先不跟。角度用 yaw_rad。 */
#include "osal/period.h"
#include "osal/osal.h"
#include "component/common/err.h"
#include "cdd/host/host.h"
#include "cdd/imu/ins/ins.h"
#ifdef CONFIG_CDD_HOST_NAV
#include "cdd/referee/referee.h"
#endif
#include "host_config.h"

#if defined(CONFIG_CDD_HOST_AIM) || defined(CONFIG_CDD_HOST_NAV)

static uint32_t tx_frames;
static uint32_t tx_fail;

static void host_send(uint8_t command, void *payload, uint8_t length)
{
    int32_t send_ret = host_transfer(command, payload, length);

    if (send_ret == ERR_NODEV)
    {
        return;
    }

    uint32_t irq_key = os_irq_save();

    if (send_ret == ERR_OK)
    {
        tx_frames++;
    }
    else
    {
        tx_fail++;
    }
    os_irq_restore(irq_key);
}

static void host_ctrl_run(void *argument)
{
    ins_attitude_t attitude;

    (void)argument;
    ins_get_attitude(&attitude);
    if (!attitude.valid)
    {
        return;
    }

#ifdef CONFIG_CDD_HOST_AIM
    {
        host_aim_attitude_t current;

        current.roll_rad = attitude.roll_rad;
        current.pitch_rad = attitude.pitch_rad;
        current.yaw_rad = attitude.yaw_rad;
        host_send(HOST_CMD_AIM_ATTITUDE, &current, sizeof current);
    }
#endif
#ifdef CONFIG_CDD_HOST_NAV
    {
        referee_info_t referee;
        host_nav_state_t nav;

        referee_get(&referee);
        nav.hp = referee.performance.current_hp;
        nav.roll_rad = attitude.roll_rad;
        nav.pitch_rad = attitude.pitch_rad;
        nav.yaw_rad = attitude.yaw_rad;
        host_send(HOST_CMD_NAV_STATE, &nav, sizeof nav);
    }
#endif
}
PERIOD_TASK_DEFINE(host_ctrl_task, OS_FREQ_100HZ, OS_PRIORITY_LOW, OS_STACK_MEDIUM, host_ctrl_run, NULL);

#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"

#ifdef CONFIG_CDD_HOST_AIM
static void print_deg(const char *label, float radian)
{
    float degree = radian * 57.2957795f;
    long milli = (long)(degree * 1000.0f + (degree >= 0.0f ? 0.5f : -0.5f));
    const char *sign = (milli < 0) ? "-" : "";

    if (milli < 0)
    {
        milli = -milli;
    }
    shell_out("  %s = %s%ld.%03ld deg\r\n", label, sign, milli / 1000, milli % 1000);
}
#endif

static int shell_host(int argc, char *argv[])
{
    host_link_t link;
#ifdef CONFIG_CDD_HOST_AIM
    host_aim_target_t target;
    int32_t target_ret;
#endif
#ifdef CONFIG_CDD_HOST_NAV
    host_nav_speed_t speed;
    int32_t speed_ret;
#endif
    uint32_t sent;
    uint32_t failed;

    (void)argc;
    (void)argv;
    host_get_link(&link);
#ifdef CONFIG_CDD_HOST_AIM
    target_ret = host_transfer(HOST_CMD_AIM_TARGET, &target, sizeof target);
#endif
#ifdef CONFIG_CDD_HOST_NAV
    speed_ret = host_transfer(HOST_CMD_NAV_SPEED, &speed, sizeof speed);
#endif

    uint32_t irq_key = os_irq_save();

    sent = tx_frames;
    failed = tx_fail;
    os_irq_restore(irq_key);

    shell_out("host: online=%d frames=%lu bad=%lu\r\n", (int)link.online,
              (unsigned long)link.frames, (unsigned long)link.bad_frames);
    shell_out("  tx=%lu tx_fail=%lu\r\n", (unsigned long)sent, (unsigned long)failed);
#ifdef CONFIG_CDD_HOST_AIM
    if (target_ret != ERR_OK)
    {
        shell_out("  aim = none\r\n");
    }
    else
    {
        print_deg("aim roll", target.roll_rad);
        print_deg("aim pitch", target.pitch_rad);
        print_deg("aim yaw", target.yaw_rad);
        shell_out("  locked=%u shoot=%u\r\n", (unsigned)target.is_locked, (unsigned)target.shoot_en);
    }
#endif
#ifdef CONFIG_CDD_HOST_NAV
    if (speed_ret != ERR_OK)
    {
        shell_out("  speed = none\r\n");
    }
    else
    {
        shell_out("  forward=%d right=%d spin=%d (mm/s, mrad/s)\r\n",
                  (int)(speed.forward_m_s * 1000.0f),
                  (int)(speed.right_m_s * 1000.0f),
                  (int)(speed.spin_rad_s * 1000.0f));
    }
#endif
    return 0;
}
SHELL_CMD(host, shell_host, show_host_link)
#endif
#endif
