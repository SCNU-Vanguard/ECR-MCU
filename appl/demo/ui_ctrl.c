/* 选手端正中画一个数字 1、一条横线和一个正方形，用来确认 UI 能发出去。
 * 坐标按 1920x1080，中心 (960, 540)。0x0301 上限 10 Hz，这里 10 Hz 交替发，两种包各 5 Hz。 */
#include "osal/period.h"
#include "component/common/err.h"
#include "cdd/referee/ui/referee_ui.h"

#define UI_CENTER_X  960u
#define UI_CENTER_Y  540u

static bool shape_added;
static bool number_added;

static void ui_ctrl_run(void *argument)
{
    static uint8_t phase;

    (void)argument;
    phase++;
    if ((phase & 1u) == 0u)
    {
        referee_ui_graph_t graph[2];
        uint32_t operate = shape_added ? REFEREE_UI_OPERATE_CHANGE : REFEREE_UI_OPERATE_ADD;

        referee_ui_line(&graph[0], "lin", operate, 1u, REFEREE_UI_COLOR_GREEN, 2u,
                        UI_CENTER_X - 100u, UI_CENTER_Y, UI_CENTER_X + 100u, UI_CENTER_Y);
        referee_ui_rectangle(&graph[1], "sqr", operate, 1u, REFEREE_UI_COLOR_GREEN, 2u,
                             UI_CENTER_X - 50u, UI_CENTER_Y - 50u, UI_CENTER_X + 50u, UI_CENTER_Y + 50u);
        if (referee_ui_graph_refresh(graph, 2u) == ERR_OK)
        {
            shape_added = true;
        }
    }
    else
    {
        referee_ui_graph_t number;
        uint32_t operate = number_added ? REFEREE_UI_OPERATE_CHANGE : REFEREE_UI_OPERATE_ADD;

        referee_ui_integer(&number, "num", operate, 1u, REFEREE_UI_COLOR_GREEN, 20u, 2u,
                           UI_CENTER_X, UI_CENTER_Y, 1);
        if (referee_ui_graph_refresh(&number, 1u) == ERR_OK)
        {
            number_added = true;
        }
    }
}
PERIOD_TASK_DEFINE(ui_ctrl_task, OS_FREQ_10HZ, OS_PRIORITY_LOW, OS_STACK_MEDIUM, ui_ctrl_run, NULL);
