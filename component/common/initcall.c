#include "component/common/initcall.h"

extern const initcall_t __start_init_region[];
extern const initcall_t __stop_init_region[];

int32_t initcall_run_all(initcall_report_fn report)
{
    int32_t failed_count = 0;

    for (const initcall_t *call = __start_init_region; call < __stop_init_region; call++)
    {
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
