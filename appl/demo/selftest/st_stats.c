/* 自测：手动触发任务统计。USB-TTL 接 USART10 的 RX（PE2），在串口助手里发送字符 's'，
 * 板子就打印一次从上次触发到现在的全部任务统计（task_stats_dump），然后清零重新统计。
 * 框架本身不会自动触发统计；这里只是给开发板一个最简单的触发方式。 */
#include "osal/event.h"
#include "bsp/uart/bsp_uart.h"
#include "component/common/initcall.h"
#include "component/common/taskstat.h"

static void stats_run(void *arg, const uint8_t *d, uint16_t n)
{
    (void)arg;
    if (d == NULL) { return; }
    for (uint16_t i = 0; i < n; i++) {
        if (d[i] == 's' || d[i] == 'S') { task_stats_dump(); break; }   /* 一批字符里只触发一次 */
    }
}
EVENT_TASK_DEFINE(st_stats, 12, 640, EVENT_QUEUE, 16, 4, 0, stats_run, NULL);

static int32_t st_stats_init(void)
{
    bsp_uart_bind_event(UART_LOG, &st_stats);
    return 0;
}
INIT_APP(st_stats_init);
