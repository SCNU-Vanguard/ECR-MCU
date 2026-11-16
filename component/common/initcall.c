#include "component/common/initcall.h"

extern const initcall_t __start_init_region[];
extern const initcall_t __stop_init_region[];

int32_t initcall_run_all(initcall_report_fn report)
{
    int32_t failed_count = 0;

    for (const initcall_t *call = __start_init_region; call < __stop_init_region; call++)
    {
        if (failed_count != 0 && call->level != call[-1].level)     /* 前一个阶段有失败：不进入下一阶段 */
        {
            break;
        }
        int32_t ret = call->fn();

        if (ret != 0)
        {
            failed_count++;
            if (report != NULL)
            {
                report(call, ret);
            }
        }
    }
    return failed_count;
}
