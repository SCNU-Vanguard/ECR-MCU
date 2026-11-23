#ifndef CDD_REFEREE_UI_REG_H__
#define CDD_REFEREE_UI_REG_H__
#include "cdd/referee/ui/referee_ui.h"

/* 选手端图案。每个兵种在 ui_config.h 里列表，八种图形的注册格式写在该文件注释里。
 * 表里的值是上电画面。应用层调用 ui_set_* 改参数，本模块 10 Hz 发一包。
 * 没改过的图只 ADD 一次，改过的下一拍 CHANGE。ui_redraw 让下一拍起全部重新 ADD。
 * 协议名 3 个字符且各行不同。图层 4 位，颜色 4 位，线宽 10 位，坐标 11 位，半径 10 位，角度和字号 9 位。
 * 调错类型的 ui_set_* 不改这张图。字符串超过 30 个字符时丢掉后面的。 */

typedef struct ui_figure ui_figure_t;

void ui_set_layer(ui_figure_t *fig, uint32_t layer);
void ui_set_color(ui_figure_t *fig, uint32_t color);
void ui_set_width(ui_figure_t *fig, uint32_t width);
void ui_set_start(ui_figure_t *fig, uint32_t x, uint32_t y);
void ui_set_end(ui_figure_t *fig, uint32_t x, uint32_t y);
void ui_set_radius(ui_figure_t *fig, uint32_t radius);
void ui_set_angle(ui_figure_t *fig, uint32_t start_deg, uint32_t end_deg);
void ui_set_font(ui_figure_t *fig, uint32_t font_size);
void ui_set_decimals(ui_figure_t *fig, uint32_t digits);
void ui_set_int(ui_figure_t *fig, int32_t value);
void ui_set_float(ui_figure_t *fig, int32_t value_x1000);
void ui_set_string(ui_figure_t *fig, const char *text);
void ui_redraw(void);

#include "ui_config.h"

#ifndef ROBOT_UI_FIGURE_TABLE
#error "ui_config.h must define ROBOT_UI_FIGURE_TABLE"
#endif

#define UI_LINE(_name, ...)    extern ui_figure_t _name;
#define UI_RECT(_name, ...)    extern ui_figure_t _name;
#define UI_CIRCLE(_name, ...)  extern ui_figure_t _name;
#define UI_ELLIPSE(_name, ...) extern ui_figure_t _name;
#define UI_ARC(_name, ...)     extern ui_figure_t _name;
#define UI_FLOAT(_name, ...)   extern ui_figure_t _name;
#define UI_INT(_name, ...)     extern ui_figure_t _name;
#define UI_STR(_name, ...)     extern ui_figure_t _name;
ROBOT_UI_FIGURE_TABLE
#undef UI_LINE
#undef UI_RECT
#undef UI_CIRCLE
#undef UI_ELLIPSE
#undef UI_ARC
#undef UI_FLOAT
#undef UI_INT
#undef UI_STR
#endif
