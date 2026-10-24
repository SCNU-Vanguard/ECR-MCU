/* 板间主机：1000 Hz 把当前请求缓存整帧发出。请求由云台任务整帧写进缓存。 */
#include "osal/period.h"
#include "cdd/interboard/interboard.h"

static void interboard_ctrl_run(void *argument)
{
    (void)argument;
    (void)interboard_post();
}
PERIOD_TASK_DEFINE(interboard_ctrl_task, OS_FREQ_1000HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, interboard_ctrl_run, NULL);
