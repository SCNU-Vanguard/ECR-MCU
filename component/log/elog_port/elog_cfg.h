#ifndef _ELOG_CFG_H_
#define _ELOG_CFG_H_

/* EasyLogger 配置。注意：只有本文件生效，上游 inc/elog_cfg.h 没有拷进仓库。 */
#define ELOG_OUTPUT_ENABLE                         /* 总开关 */
#define ELOG_OUTPUT_LVL                  ELOG_LVL_VERBOSE   /* 编译期最低输出级别 */
#define ELOG_ASSERT_ENABLE                         /* 允许 log_a / ELOG_ASSERT */
#define ELOG_LINE_BUF_SIZE               256       /* 一行日志的最大长度（静态缓冲） */
#define ELOG_LINE_NUM_MAX_LEN            5
#define ELOG_FILTER_TAG_MAX_LEN          16
#define ELOG_FILTER_KW_MAX_LEN           16
#define ELOG_FILTER_TAG_LVL_MAX_NUM      5
#define ELOG_NEWLINE_SIGN                "\r\n"
/* 不用颜色、不用缓冲输出、不用异步输出：保持同步输出，bsp_uart_write 本身不阻塞 */
/* #define ELOG_COLOR_ENABLE */
/* #define ELOG_BUF_OUTPUT_ENABLE */
/* #define ELOG_ASYNC_OUTPUT_ENABLE */

#endif
