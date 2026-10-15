/* EasyLogger 的移植层：把输出、锁、时间接到本框架 */
#include <stdio.h>
#include "elog.h"
#include "osal/osal.h"
#include "bsp/tick/bsp_tick.h"
#include "bsp/include/bsp_ids.h"
#include "bsp/uart/bsp_uart.h"
#include "component/common/initcall.h"
#include "component/common/err.h"

static os_mutex_t output_lock;
static bool       output_lock_ready;
static char       time_buffer[16];

ElogErrCode elog_port_init(void)
{
    if (os_mutex_init(&output_lock) == ERR_OK)
    {
        output_lock_ready = true;
    }
    return ELOG_NO_ERR;
}

void elog_port_deinit(void)
{
}

/* 输出一整行：交给串口发送队列，不阻塞 */
void elog_port_output(const char *log, size_t size)
{
    (void)bsp_uart_write(UART_LOG, log, (uint32_t)size);
}

/* 输出锁：多个任务同时打印时保证一行日志不被穿插。调度器启动前、中断里不加锁（中断里本来就不允许打日志） */
void elog_port_output_lock(void)
{
    if (output_lock_ready && os_running() && !os_in_isr())
    {
        (void)os_mutex_lock(&output_lock, OS_WAIT_FOREVER);
    }
}

void elog_port_output_unlock(void)
{
    if (output_lock_ready && os_running() && !os_in_isr())
    {
        os_mutex_unlock(&output_lock);
    }
}

/* 时间戳：自启动时间，秒.毫秒，如 "45.724"（45 秒 724 毫秒，毫秒位向下取整，不带微秒）。
 * 返回的是静态缓冲，EasyLogger 在持有输出锁时调用它 */
const char *elog_port_get_time(void)
{
    uint32_t time_ms = bsp_tick_ms();

    (void)snprintf(time_buffer, sizeof time_buffer, "%lu.%03lu",
                   (unsigned long)(time_ms / 1000u), (unsigned long)(time_ms % 1000u));
    return time_buffer;
}

/* 进程信息：不用 */
const char *elog_port_get_p_info(void)
{
    return "";
}

/* 线程信息：不用 */
const char *elog_port_get_t_info(void)
{
    return "";
}

/* 初始化：INIT_COMP_STAGE1，在 BSP 的 UART（INIT_BSP_STAGE1）之后 */
static int32_t log_init(void)
{
    elog_init();                                           /* 内部会调 elog_port_init() */
    for (uint8_t level = ELOG_LVL_ASSERT; level <= ELOG_LVL_VERBOSE; level++)
    {
        /* 每行格式：[级别]<时间>正文，如 [INFO]<45.724>framework up（时间 = 秒.毫秒）。不输出标签、文件名、函数名、行号 */
        elog_set_fmt(level, ELOG_FMT_LVL | ELOG_FMT_TIME);
    }
    elog_start();
    return ERR_OK;
}
INIT_COMP_STAGE1(log_init);
