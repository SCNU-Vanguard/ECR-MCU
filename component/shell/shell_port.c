/* letter-shell 的移植层：输入来自 USART10 的接收事件，输出走 bsp_uart_write
 * 日志和 shell 共用 USART10，两边都是整块写进发送队列，各自完整，只是可能一行日志插在两次 shell 输出之间。 */
#include "shell.h"
#include "osal/event.h"
#include "bsp/include/bsp_ids.h"
#include "bsp/uart/bsp_uart.h"
#include "component/common/initcall.h"

static Shell shell;
static char  shell_buffer[512];                 /* letter-shell 的命令行缓冲 */

static signed short shell_write(char *data, unsigned short length)
{
    return (signed short)bsp_uart_write(UART_LOG, data, length);
}

/* 事件任务：USART10 收到的字符块（≤ 16 字节，等于接收缓冲）逐字符喂给 shell。
 * 命令函数就在这个任务里执行，所以栈要给够；命令里不要长时间阻塞 */
static void shell_run(void *argument, const uint8_t *data, uint16_t length)
{
    (void)argument;
    if (data == NULL)
    {
        return;
    }
    for (uint16_t index = 0; index < length; index++)
    {
        shellHandler(&shell, (char)data[index]);
    }
}
EVENT_TASK_DEFINE(shell_task, 12, 768, EVENT_QUEUE, 16, 8, 0, shell_run, NULL);

static int32_t shell_port_init(void)
{
    shell.write = shell_write;
    shellInit(&shell, shell_buffer, sizeof shell_buffer);
    bsp_uart_bind_event(UART_LOG, &shell_task);
    return 0;
}
INIT_APP(shell_port_init);
