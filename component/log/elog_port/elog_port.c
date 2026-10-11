/* EasyLogger 的移植层：把输出、锁、时间接到本框架 */
#include <stdio.h>
#include "elog.h"
#include "osal/osal.h"
#include "bsp/include/bsp_ids.h"
#include "bsp/uart/bsp_uart.h"
#include "component/common/initcall.h"
#include "component/common/err.h"

static os_mutex_t lock;
static bool       lock_ok;
static char       time_buf[16];

ElogErrCode elog_port_init(void)
{
    if (os_mutex_init(&lock) == ERR_OK) { lock_ok = true; }
    return ELOG_NO_ERR;
}

void elog_port_deinit(void) {}

/* 输出一整行：交给串口发送队列，不阻塞 */
void elog_port_output(const char *log, size_t size)
{
    (void)bsp_uart_write(UART_LOG, log, (uint32_t)size);
}

/* 输出锁：多个任务同时打印时保证一行日志不被穿插。调度器启动前、中断里不加锁（中断里本来就不允许打日志） */
void elog_port_output_lock(void)
{
    if (lock_ok && os_running() && !os_in_isr()) { (void)os_mutex_lock(&lock, OS_WAIT_FOREVER); }
}

void elog_port_output_unlock(void)
{
    if (lock_ok && os_running() && !os_in_isr()) { os_mutex_unlock(&lock); }
}

/* 时间戳：自启动毫秒数。返回的是静态缓冲，EasyLogger 在持有输出锁时调用它 */
const char *elog_port_get_time(void)
{
    (void)snprintf(time_buf, sizeof time_buf, "%lu", (unsigned long)os_time_ms());
    return time_buf;
}

const char *elog_port_get_p_info(void) { return ""; }     /* 进程信息：不用 */
const char *elog_port_get_t_info(void) { return ""; }     /* 线程信息：不用 */

/* 初始化：INIT_COMP 级别，在 bsp 的 UART 之后 */
static int32_t log_init(void)
{
    elog_init();                                           /* 内部会调 elog_port_init() */
    for (uint8_t lvl = ELOG_LVL_ASSERT; lvl <= ELOG_LVL_VERBOSE; lvl++) {
        /* 每行开头：级别、标签、时间。不要文件名/函数名/行号，省输出量 */
        elog_set_fmt(lvl, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    }
    elog_start();
    return ERR_OK;
}
INIT_COMP(log_init);
