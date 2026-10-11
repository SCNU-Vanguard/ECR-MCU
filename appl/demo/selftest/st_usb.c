/* 自测：USB CDC 回显。PC 往虚拟串口发什么，板子原样发回什么。PC 端脚本见 tools/echo_test.py。
 * bsp_usb 在 USB 中断里把收到的包（≤ 64 字节）拷进事件队列，任务里原样写回去。 */
#include "osal/event.h"
#include "bsp/usb/bsp_usb.h"
#include "component/common/initcall.h"

static void usb_echo_run(void *arg, const uint8_t *d, uint16_t n)
{
    (void)arg;
    if (d != NULL) { (void)bsp_usb_write(d, n); }
}
EVENT_TASK_DEFINE(st_usb_echo, 9, 384, EVENT_QUEUE, 64, 8, 0, usb_echo_run, NULL);

static int32_t st_usb_init(void)
{
    bsp_usb_bind_event(&st_usb_echo);
    return 0;
}
INIT_APP(st_usb_init);
