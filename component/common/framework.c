#include "component/common/framework.h"
#include "component/common/initcall.h"
#include "component/log/log.h"
#include "osal/period.h"
#include "osal/event.h"
#include "osal/task_stats.h"

static void report_fail(const initcall_t *call, int32_t ret)
{
    LOGE("init", "%s failed: %d", call->name, (int)ret);
}

void framework_init(void)
{
    task_stats_hw_init();                                           /* 0. 打开周期计数器（任务统计用） */
    int32_t event_start_ret = event_start_all();                    /* 1. 先创建全部事件任务的队列：后面初始化里的中断就可以往里投递 */
    int32_t failed_count = initcall_run_all(report_fail);           /* 2. 分 8 个阶段初始化：BSP1/2 -> COMP1/2 -> CDD1/2 -> APP1/2 */
    int32_t period_start_ret = period_start_all();                  /* 3. 创建全部周期任务，然后 tick 才开始放行 */

    LOGI("init", "framework up, initcall failed=%d, event start=%d, period start=%d",
         (int)failed_count, (int)event_start_ret, (int)period_start_ret);
}
