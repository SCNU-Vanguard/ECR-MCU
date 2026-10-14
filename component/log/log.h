#ifndef LOG_H__
#define LOG_H__

/* log 模块：由 EasyLogger 实现（第 19.4 节）。
 * LOGx(tag, fmt, ...) 底层转成 elog_output；中断里调用直接丢弃（EasyLogger 用静态行缓冲 + 互斥量，不能在中断里用）。
 * CONFIG_COMPONENT_LOG=n 时 LOGx 展开为空，调用方不用改。 */
#ifdef CONFIG_COMPONENT_LOG
#include "osal/osal.h"
#include "elog.h"
#define LOG_EL(level, tag, fmt, ...) \
    (os_in_isr() ? (void)0 : elog_output((level), (tag), "", "", 0, fmt, ##__VA_ARGS__))
#define LOGE(tag, fmt, ...)  LOG_EL(ELOG_LVL_ERROR, tag, fmt, ##__VA_ARGS__)
#define LOGW(tag, fmt, ...)  LOG_EL(ELOG_LVL_WARN,  tag, fmt, ##__VA_ARGS__)
#define LOGI(tag, fmt, ...)  LOG_EL(ELOG_LVL_INFO,  tag, fmt, ##__VA_ARGS__)
#else
/* 空实现仍然“使用”参数，避免调用处出现 unused variable 告警，并保留格式检查 */
static inline void log_discard(const char *tag, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static inline void log_discard(const char *tag, const char *fmt, ...)
{
    (void)tag;
    (void)fmt;
}
#define LOGE(tag, fmt, ...)  log_discard(tag, fmt, ##__VA_ARGS__)
#define LOGW(tag, fmt, ...)  log_discard(tag, fmt, ##__VA_ARGS__)
#define LOGI(tag, fmt, ...)  log_discard(tag, fmt, ##__VA_ARGS__)
#endif
#endif
