#include "component/common/initcall.h"

extern const initcall_t __start_init_region[];
extern const initcall_t __stop_init_region[];

int32_t initcall_run_all(initcall_report_fn report)
{
    int32_t failed = 0;
    for (const initcall_t *c = __start_init_region; c < __stop_init_region; c++) {
        int32_t ret = c->fn();
        if (ret != 0) {
            failed++;
            if (report != NULL) { report(c, ret); }
        }
    }
    return failed;
}
