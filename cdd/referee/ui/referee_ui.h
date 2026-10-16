#ifndef CDD_REFEREE_UI_H__
#define CDD_REFEREE_UI_H__
#include "cdd/referee/referee.h"

/* 选手端 UI 绘图。分两步：先往图形结构里填内容（referee_ui_line 等），再 referee_ui_graph_refresh 发出去。
 * 接收方是本机器人的选手端，ID 从 0x0201 里取；还没收到时发送函数返回 ERR_BUSY。
 * 0x0301 整体上行频率上限 10 Hz（删除、图形、字符串、机器人间通信共用），这里不限速。
 *
 * 图形名字 3 个字符，按原 UI 库的顺序填：第 1 个字符放在 name[2]。已有界面的名字是按这个顺序对上的。 */

/* 直线。start/end 是两端点 */
void referee_ui_line(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                     uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);

/* 矩形。start/end 是对角顶点 */
void referee_ui_rectangle(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                          uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);

/* 圆。start 是圆心 */
void referee_ui_circle(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                       uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t radius);

/* 椭圆。start 是圆心，end_x/end_y 是 x/y 半轴长 */
void referee_ui_ellipse(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                        uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);

/* 圆弧。start 是圆心，end_x/end_y 是 x/y 半轴长，角度单位是度 */
void referee_ui_arc(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                    uint32_t start_angle, uint32_t end_angle, uint32_t width, uint32_t start_x, uint32_t start_y,
                    uint32_t end_x, uint32_t end_y);

/* 浮点。value 是数值乘 1000 后的整数（协议把 32 位整数拆进图形结构的 radius/end_x/end_y） */
void referee_ui_float(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                      uint32_t font_size, uint32_t decimal_digits, uint32_t width, uint32_t start_x, uint32_t start_y,
                      int32_t value);

/* 整数。value 同样拆进 radius/end_x/end_y */
void referee_ui_integer(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                        uint32_t font_size, uint32_t width, uint32_t start_x, uint32_t start_y, int32_t value);

/* 字符串，printf 风格。最多 30 个字符，多出来的丢掉 */
void referee_ui_string(referee_ui_string_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                       uint32_t font_size, uint32_t width, uint32_t start_x, uint32_t start_y, const char *fmt, ...);

/* 删除。operate 是 referee_ui_delete_e；删全部图层时 layer 无意义。返回值同 referee_send_frame */
int32_t referee_ui_delete(uint8_t operate, uint8_t layer);

/* 把填好的图形发出去。count 只能是 1、2、5、7，否则返回 ERR_INVAL。graphs 是数组，不是可变参数 */
int32_t referee_ui_graph_refresh(const referee_ui_graph_t *graphs, uint32_t count);

/* 把填好的字符串发出去 */
int32_t referee_ui_string_refresh(const referee_ui_string_t *graph);
#endif
