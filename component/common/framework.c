#include "component/common/framework.h"
#include "component/common/initcall.h"
#include "component/log/log.h"
#include "osal/period.h"
#include "osal/event.h"
#include "osal/task_stats.h"

static void report_fail(const initcall_t *c, int32_t ret)
{
    LOGE("init", "%s failed: %d", c->name, (int)ret);
}

void framework_init(void)
{
    task_stats_hw_init();                                /* 0. 打开周期计数器（任务统计用） */
    int32_t re = event_start_all();                      /* 1. 先创建全部事件任务的队列：后面初始化里的中断就可以往里投递 */
    int32_t failed = initcall_run_all(report_fail);      /* 2. 分级初始化：bsp -> 组件 -> 设备 -> 应用 */
    int32_t r = period_start_all();                      /* 3. 创建全部周期任务，然后 tick 才开始放行 */
    LOGI("init", "framework up, initcall failed=%d, event start=%d, period start=%d", (int)failed, (int)re, (int)r);
}
