#ifndef UI_CONFIG_H__
#define UI_CONFIG_H__
/* demo 选手端正中一条横线、一个正方形、数字 1。坐标按 1920x1080，中心 (960, 540)。
 * 对象名是 C 变量，协议名是 3 个字符，选手端靠协议名认这张图，各行不能重复。
 * 本表里的对象：center_line、center_rect、center_number。
 *
 * UI_LINE   (对象名, 协议名, 图层, 颜色, 线宽, x0, y0, x1, y1)
 * UI_RECT   (对象名, 协议名, 图层, 颜色, 线宽, x0, y0, x1, y1)
 * UI_CIRCLE (对象名, 协议名, 图层, 颜色, 线宽, 圆心x, 圆心y, 半径)
 * UI_ELLIPSE(对象名, 协议名, 图层, 颜色, 线宽, 圆心x, 圆心y, 半轴x, 半轴y)
 * UI_ARC    (对象名, 协议名, 图层, 颜色, 起始角, 终止角, 线宽, 圆心x, 圆心y, 半轴x, 半轴y)
 * UI_FLOAT  (对象名, 协议名, 图层, 颜色, 字号, 小数位, 线宽, x, y, 数值乘1000)
 * UI_INT    (对象名, 协议名, 图层, 颜色, 字号, 线宽, x, y, 数值)
 * UI_STR    (对象名, 协议名, 图层, 颜色, 字号, 线宽, x, y, 文本) */
#define UI_CENTER_X  960u
#define UI_CENTER_Y  540u

#define ROBOT_UI_FIGURE_TABLE \
    UI_LINE(center_line,   "lin", 1, REFEREE_UI_COLOR_GREEN, 2,  UI_CENTER_X - 100u, UI_CENTER_Y,       UI_CENTER_X + 100u, UI_CENTER_Y      ) \
    UI_RECT(center_rect,   "sqr", 1, REFEREE_UI_COLOR_GREEN, 2,  UI_CENTER_X - 50u,  UI_CENTER_Y - 50u, UI_CENTER_X + 50u,  UI_CENTER_Y + 50u) \
    UI_INT (center_number, "num", 1, REFEREE_UI_COLOR_GREEN, 20, 2, UI_CENTER_X, UI_CENTER_Y, 1)
#endif
