/* 自测：DBUS。没有遥控器时用 PC 模拟（tools/dt7_sim.py，USB-TTL 接 UART5 的 RX 脚 PD2）。
 * 每秒打印一次解析结果和帧计数；frames 增长而 bad 不增长 = 解析通过。 */
#include "osal/period.h"
#include "cdd/remote/dt7.h"
#include "component/log/log.h"

static void dbus_report(void *arg)
{
    dt7_data_t d;
    (void)arg;
    dt7_get(&d);
    LOGI("st_dbus", "online=%d frames=%lu bad=%lu ch=%d,%d,%d,%d wheel=%d s=%u/%u key=0x%04X",
         d.online, (unsigned long)d.frames, (unsigned long)d.bad_frames,
         d.ch[0], d.ch[1], d.ch[2], d.ch[3], d.wheel, d.s1, d.s2, d.key);
}
PERIOD_TASK_DEFINE(st_dbus, 1, 18, 384, dbus_report, NULL);
