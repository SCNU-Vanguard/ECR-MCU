/* shell 命令。命令用 SHELL_EXPORT_CMD 导出，进链接段 shellCommand，不需要在别处登记。 */
#include <string.h>
#include "shell.h"
#include "component/common/taskstat.h"
#ifdef CONFIG_LOG
#include "elog.h"
#endif

/* stats：打印任务统计并清零（第 8.6 节） */
static int cmd_stats(int argc, char *argv[])
{
    (void)argc; (void)argv;
    task_stats_dump();
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), stats, cmd_stats, task_stats_dump_and_clear);

#ifdef CONFIG_LOG
/* log：运行时调整日志过滤。用法：
 *   log lvl <e|w|i|d|v>   只输出不低于该级别的日志
 *   log tag <名字>        只输出该标签（log tag 不带参数 = 取消过滤）
 *   log kw <关键字>       只输出包含关键字的日志（不带参数 = 取消）
 *   log on | off          总开关 */
static int cmd_log(int argc, char *argv[])
{
    if (argc >= 3 && strcmp(argv[1], "lvl") == 0) {
        static const char order[] = "ewidv";                      /* 与 ELOG_LVL_ERROR..VERBOSE 对应 */
        const char *p = strchr(order, argv[2][0]);
        if (p == NULL) { shellPrint(shellGetCurrent(), "level: e w i d v\r\n"); return -1; }
        elog_set_filter_lvl((uint8_t)(ELOG_LVL_ERROR + (p - order)));
    } else if (argc >= 2 && strcmp(argv[1], "tag") == 0) {
        elog_set_filter_tag((argc >= 3) ? argv[2] : "");
    } else if (argc >= 2 && strcmp(argv[1], "kw") == 0) {
        elog_set_filter_kw((argc >= 3) ? argv[2] : "");
    } else if (argc >= 2 && strcmp(argv[1], "on") == 0) {
        elog_set_output_enabled(true);
    } else if (argc >= 2 && strcmp(argv[1], "off") == 0) {
        elog_set_output_enabled(false);
    } else {
        shellPrint(shellGetCurrent(), "usage: log lvl <e|w|i|d|v> | tag [name] | kw [word] | on | off\r\n");
        return -1;
    }
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), log, cmd_log, log_filter_control);
#endif
