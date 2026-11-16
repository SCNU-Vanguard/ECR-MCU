#include "component/common/framework.h"
#include "component/common/initcall.h"
#include "component/log/log.h"
#include "osal/osal.h"
#include "osal/period.h"
#include "osal/event.h"

#define FAIL_RECORD_MAX     8u

typedef struct
{
    const char *name;
    int32_t     ret;
    uint8_t     level;
} fail_record_t;

static fail_record_t fail_records[FAIL_RECORD_MAX];             /* 日志在 COMP1 才启动，BSP 阶段的失败先记下，之后重复打印 */
static uint32_t fail_total;

static const char *const level_names[] =
{
    "BSP1", "BSP2", "COMP1", "COMP2", "CDD1", "CDD2", "APP1", "APP2"
};

static void report_fail(const initcall_t *call, int32_t ret)
{
    if (fail_total < FAIL_RECORD_MAX)
    {
        fail_records[fail_total].name = call->name;
        fail_records[fail_total].ret = ret;
        fail_records[fail_total].level = call->level;
    }
    fail_total++;
    LOGE("%s failed: %d", call->name, (int)ret);                /* 日志未启动时是空操作，靠 fail_records 补报 */
}

static void report_fail_loop(int32_t event_start_ret)
{
#ifdef CONFIG_COMPONENT_LOG
    log_ensure_started();                                       /* BSP 阶段失败时 COMP1 没跑过：补启动日志，否则后面的报告全丢 */
#endif
    for (;;)
    {
        uint32_t count = (fail_total < FAIL_RECORD_MAX) ? fail_total : FAIL_RECORD_MAX;

        for (uint32_t index = 0; index < count; index++)
        {
            const fail_record_t *record = &fail_records[index];
            const char *stage = (record->level < 8u) ? level_names[record->level] : "?";

            LOGE("init failed: [%s] %s ret=%d", stage, record->name, (int)record->ret);
        }
        LOGE("init failed total=%u, event start=%d, periodic tasks NOT started",
             (unsigned)fail_total, (int)event_start_ret);
        os_delay_ms(1000u);
    }
}

void framework_init(void)
{
    int32_t event_start_ret = event_start_all();                    /* 1. 先创建全部事件任务的队列：后面初始化里的中断就可以往里投递 */
    int32_t failed_count = initcall_run_all(report_fail);           /* 2. 分 8 个阶段初始化：BSP1/2 -> COMP1/2 -> CDD1/2 -> APP1/2，某阶段失败则不再进入后面的阶段 */

    if (failed_count != 0)
    {
        report_fail_loop(event_start_ret);                          /* 3. 失败：不启动周期任务，本任务留在这里每 1 s 重报 */
    }
    int32_t period_start_ret = period_start_all();                  /* 3. 创建全部周期任务，然后 tick 才开始放行 */

    LOGI("framework up, initcall failed=%d, event start=%d, period start=%d",
         (int)failed_count, (int)event_start_ret, (int)period_start_ret);
}
