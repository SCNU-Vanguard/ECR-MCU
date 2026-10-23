/* 板间主机：500 Hz 把当前请求缓存整帧发出。控制块和发射块由各自的任务填。 */
#include "osal/period.h"
#include "cdd/interboard/interboard.h"

static void interboard_ctrl_run(void *argument)
{
    (void)argument;
    (void)interboard_post();
}
PERIOD_TASK_DEFINE(interboard_ctrl_task, OS_FREQ_500HZ, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, interboard_ctrl_run, NULL);
