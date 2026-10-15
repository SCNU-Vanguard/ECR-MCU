#ifndef APPL_DEMO_HEATER_CTRL_H__
#define APPL_DEMO_HEATER_CTRL_H__
#include <stdbool.h>

/* 温度输入是否已经失效（INS 没有新数据），失效时加热已被关闭 */
bool heater_ctrl_input_stale(void);
#endif