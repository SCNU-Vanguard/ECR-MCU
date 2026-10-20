/* demo 视觉：100 Hz 上传当前姿态。上位机下发的目标角只存下来，云台先不跟。
 * 角度用 yaw_rad（包在 ±pi），不用累计的 yaw_total_rad。 */
#include <string.h>
#include "osal/period.h"
#include "osal/osal.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "cdd/vision/vision.h"
#include "cdd/imu/ins/ins.h"
#include "vision_config.h"

_Static_assert(sizeof(vision_angles_t) == 12u, "vision angles must be 3 little-endian floats");

static vision_angles_t target;
static bool target_seen;
static uint32_t target_frames;
static uint32_t ignored_frames;
static uint32_t tx_frames;
static uint32_t tx_fail;

static void vision_on_rx(uint8_t command, const uint8_t *payload, uint8_t length, void *argument)
{
    (void)argument;
    if (command != VISION_CMD_TARGET || length != sizeof target || payload == NULL)
    {
        uint32_t irq_key = os_irq_save();

        ignored_frames++;
        os_irq_restore(irq_key);
        return;
    }

    vision_angles_t next;

    memcpy(&next, payload, sizeof next);

    uint32_t irq_key = os_irq_save();

    target = next;
    target_seen = true;
    target_frames++;
    os_irq_restore(irq_key);
}

static int32_t vision_ctrl_init(void)
{
    vision_bind_rx(vision_on_rx, NULL);
    return ERR_OK;
}
INIT_APP_STAGE2(vision_ctrl_init);

static void vision_ctrl_run(void *argument)
{
    ins_attitude_t attitude;

    (void)argument;
    ins_get_attitude(&attitude);
    if (!attitude.valid)
    {
        return;
    }

    vision_angles_t current =
    {
        .roll_rad = attitude.roll_rad,
        .pitch_rad = attitude.pitch_rad,
        .yaw_rad = attitude.yaw_rad,
    };
    int32_t send_ret = vision_send(VISION_CMD_ATTITUDE, &current, sizeof current);

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
PERIOD_TASK_DEFINE(vision_ctrl_task, OS_FREQ_100HZ, OS_PRIORITY_LOW, OS_STACK_SMALL, vision_ctrl_run, NULL);

#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"

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

static int shell_vision(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    vision_link_t link;
    vision_angles_t last_target;
    bool seen;
    uint32_t targets;
    uint32_t ignored;
    uint32_t sent;
    uint32_t failed;

    vision_get_link(&link);

    uint32_t irq_key = os_irq_save();

    last_target = target;
    seen = target_seen;
    targets = target_frames;
    ignored = ignored_frames;
    sent = tx_frames;
    failed = tx_fail;
    os_irq_restore(irq_key);

    shell_out("vision: online=%d frames=%lu bad=%lu\r\n", (int)link.online,
              (unsigned long)link.frames, (unsigned long)link.bad_frames);
    shell_out("  tx=%lu tx_fail=%lu target_frames=%lu ignored=%lu\r\n",
              (unsigned long)sent, (unsigned long)failed, (unsigned long)targets, (unsigned long)ignored);
    if (!seen)
    {
        shell_out("  target = none\r\n");
        return 0;
    }
    print_deg("target roll", last_target.roll_rad);
    print_deg("target pitch", last_target.pitch_rad);
    print_deg("target yaw", last_target.yaw_rad);
    return 0;
}
SHELL_CMD(vision, shell_vision, show_vision_link)
#endif
