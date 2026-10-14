#ifndef SHELL_CMD_H__
#define SHELL_CMD_H__
/* 给各模块注册 shell 命令用的头文件。
 * 约定：每个模块在自己的目录里写 xxx_shell.c，用 SHELL_CMD 注册自己的命令，
 * 并在自己的 module.mk 里写 obj-$(CONFIG_SHELL) += $(d)xxx_shell.o（模块没开、shell 没开都不会编译）。
 * shell 模块不依赖任何业务模块，命令再多也不会往 component/shell 里引依赖。
 *
 * 用法：
 *   static int shell_xxx(int argc, char *argv[])    // argv[0] 是命令名
 *   (函数体按普通函数写，返回 0 表示成功)
 *   SHELL_CMD(xxx, shell_xxx, 一句话说明)             // 说明不用加引号，末尾不要加分号
 * 命令在 shell_task 事件任务里执行，不要长时间阻塞；输出用 shell_out()，不要用 LOGx。 */
#include "shell.h"

/* SHELL_EXPORT_CMD 里会把 int(*)(int,char**) 强转成 int(*)()，GCC 会报 -Wcast-function-type，这里局部屏蔽 */
#define SHELL_CMD(_name, _func, _desc)                                                  \
    _Pragma("GCC diagnostic push")                                                      \
    _Pragma("GCC diagnostic ignored \"-Wcast-function-type\"")                          \
    SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),     \
                     _name, _func, _desc);                                              \
    _Pragma("GCC diagnostic pop")

/* 向当前 shell 输出（printf 风格） */
#define shell_out(...)  shellPrint(shellGetCurrent(), __VA_ARGS__)
#endif
