/* 自测：串口回显。PC 通过 USB-TTL / USB-RS485 适配器发什么，板子原样发回什么。
 * 覆盖 UART_REFEREE(USART1)、UART_INTERBOARD(USART2, RS485)、UART_IMGTRANS(UART7)，
 * 同时检验 DMA/中断接收、事件任务、发送队列、RS485 方向脚。PC 端脚本见 tools/echo_test.py。
 * 每个串口一个事件任务：bsp_uart 在中断里把收到的数据块拷进事件队列，任务里原样写回去。 */
#include "osal/event.h"
#include "bsp/uart/bsp_uart.h"
#include "component/common/initcall.h"
#include "component/log/log.h"

/* max_len 必须不小于该串口的接收缓冲（board.c 里：referee 256、inter 128、imgtrans 512） */
#define ECHO_TASK(_name, _id, _maxlen)                                                         \
    static void _name##_run(void *arg, const uint8_t *d, uint16_t n)                           \
    { (void)arg; if (d != NULL) { (void)bsp_uart_write(_id, d, n); } }                         \
    EVENT_TASK_DEFINE(_name, 8, 384, EVENT_QUEUE, _maxlen, 4, 0, _name##_run, NULL)

ECHO_TASK(st_echo_referee, UART_REFEREE,    256);
ECHO_TASK(st_echo_inter,   UART_INTERBOARD, 128);
ECHO_TASK(st_echo_imgtr,   UART_IMGTRANS,   512);

static int32_t st_uart_init(void)
{
    bsp_uart_bind_event(UART_REFEREE,    &st_echo_referee);
    bsp_uart_bind_event(UART_INTERBOARD, &st_echo_inter);
    bsp_uart_bind_event(UART_IMGTRANS,   &st_echo_imgtr);
    return 0;
}
INIT_APP(st_uart_init);

/* 每 2 秒报一次计数：rx/tx 字节数、未绑定丢弃、发送丢弃、HAL 错误次数。
 * 事件队列自己的 post/done/drop 在任务统计里（发字符 s 触发，第 8.6 节） */
#include "osal/period.h"
static const uart_id_t ports[] = { UART_REFEREE, UART_INTERBOARD, UART_IMGTRANS };
static const char *const names[] = { "referee", "interboard", "imgtrans" };

static void report_task(void *arg)
{
    static uint32_t n;
    (void)arg;
    if (++n % 2u != 0u) { return; }
    for (unsigned i = 0; i < ARRAY_SIZE(ports); i++) {
        const bsp_uart_state_t *s = bsp_uart_state(ports[i]);
        LOGI("st_uart", "%-10s rx=%lu tx=%lu unbound=%lu tx_drop=%lu err=%lu", names[i],
             (unsigned long)s->rx_bytes, (unsigned long)s->tx_bytes,
             (unsigned long)s->rx_unbound, (unsigned long)s->tx_drop, (unsigned long)s->err_cnt);
    }
}
PERIOD_TASK_DEFINE(st_uart_report, 1, 19, 384, report_task, NULL);
