/* 选手端正中画一个数字 1、一条横线和一个正方形，用来确认 UI 能发出去。
 * 坐标按 1920x1080，中心 (960, 540)。0x0301 上限 10 Hz，这里 10 Hz 交替发，两种包各 5 Hz。
 * 上电先 ADD 一次，之后 CHANGE。选手端刷新或重连后图形丢了，操作手拨一下 custom4（状态变化一次）
 * 就会把全部图形重新 ADD 一遍。 */
#include "osal/period.h"
#include "component/common/err.h"
#include "cdd/operate/operate.h"
#include "cdd/referee/ui/referee_ui.h"

#define UI_CENTER_X  960u
#define UI_CENTER_Y  540u

static bool shape_added;
static bool number_added;
static bool last_custom4;

/* custom4 状态变化一次，下一次发送重新 ADD。 */
static void ui_check_refresh(void)
{
    operate_snap_t op = {0};

    (void)operate_get_snapshot(&op);
    if (op.custom4 != last_custom4)
    {
        last_custom4 = op.custom4;
        shape_added = false;
        number_added = false;
    }
}

static void ui_ctrl_run(void *argument)
{
    static uint8_t phase;

    (void)argument;
    ui_check_refresh();
    phase++;
    if ((phase & 1u) == 0u)
    {
        referee_ui_graph_t graph[2];
        uint32_t graph_operate = shape_added ? REFEREE_UI_OPERATE_CHANGE : REFEREE_UI_OPERATE_ADD;

        referee_ui_line(&graph[0], "lin", graph_operate, 1u, REFEREE_UI_COLOR_GREEN, 2u,
                        UI_CENTER_X - 100u, UI_CENTER_Y, UI_CENTER_X + 100u, UI_CENTER_Y);
        referee_ui_rectangle(&graph[1], "sqr", graph_operate, 1u, REFEREE_UI_COLOR_GREEN, 2u,
                             UI_CENTER_X - 50u, UI_CENTER_Y - 50u, UI_CENTER_X + 50u, UI_CENTER_Y + 50u);
        if (referee_ui_graph_refresh(graph, 2u) == ERR_OK)
        {
            shape_added = true;
        }
    }
    else
    {
        referee_ui_graph_t number;
        uint32_t graph_operate = number_added ? REFEREE_UI_OPERATE_CHANGE : REFEREE_UI_OPERATE_ADD;

        referee_ui_integer(&number, "num", graph_operate, 1u, REFEREE_UI_COLOR_GREEN, 20u, 2u,
                           UI_CENTER_X, UI_CENTER_Y, 1);
        if (referee_ui_graph_refresh(&number, 1u) == ERR_OK)
        {
            number_added = true;
        }
    }
}
PERIOD_TASK_DEFINE(ui_ctrl_task, OS_FREQ_10HZ, OS_PRIORITY_LOW, OS_STACK_MEDIUM, ui_ctrl_run, NULL);
