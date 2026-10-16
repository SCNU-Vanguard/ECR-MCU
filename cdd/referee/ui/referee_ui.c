#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "cdd/referee/ui/referee_ui.h"
#include "component/common/err.h"

/* 名字按原 UI 库的顺序填，见 referee_ui.h */
static void set_name(uint8_t name[3], const char *graph_name)
{
    if (graph_name == NULL)
    {
        return;
    }
    for (int i = 0; i < 3 && graph_name[i] != '\0'; i++)
    {
        name[2 - i] = (uint8_t)graph_name[i];
    }
}

static void set_common(referee_ui_graph_t *graph, const char *name, uint32_t operate, uint32_t graphic_type,
                       uint32_t layer, uint32_t color)
{
    memset(graph, 0, sizeof *graph);
    set_name(graph->graphic_name, name);
    graph->operate_type = operate;
    graph->graphic_type = graphic_type;
    graph->layer = layer;
    graph->color = color;
}

/* 整数和浮点的数值拆进图形结构末尾三个位域：低 10 位、中 11 位、高 11 位 */
static void set_number(referee_ui_graph_t *graph, int32_t value)
{
    uint32_t bits = (uint32_t)value;

    graph->radius = bits & 0x3FFu;
    graph->end_x = (bits >> 10) & 0x7FFu;
    graph->end_y = (bits >> 21) & 0x7FFu;
}

void referee_ui_line(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                     uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y)
{
    set_common(graph, name, operate, REFEREE_UI_GRAPH_LINE, layer, color);
    graph->width = width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

void referee_ui_rectangle(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                          uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y)
{
    set_common(graph, name, operate, REFEREE_UI_GRAPH_RECTANGLE, layer, color);
    graph->width = width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

void referee_ui_circle(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                       uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t radius)
{
    set_common(graph, name, operate, REFEREE_UI_GRAPH_CIRCLE, layer, color);
    graph->width = width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    graph->radius = radius;
}

void referee_ui_ellipse(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                        uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y)
{
    set_common(graph, name, operate, REFEREE_UI_GRAPH_ELLIPSE, layer, color);
    graph->width = width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

void referee_ui_arc(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                    uint32_t start_angle, uint32_t end_angle, uint32_t width, uint32_t start_x, uint32_t start_y,
                    uint32_t end_x, uint32_t end_y)
{
    set_common(graph, name, operate, REFEREE_UI_GRAPH_ARC, layer, color);
    graph->start_angle = start_angle;
    graph->end_angle = end_angle;
    graph->width = width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

void referee_ui_float(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                      uint32_t font_size, uint32_t decimal_digits, uint32_t width, uint32_t start_x, uint32_t start_y,
                      int32_t value)
{
    set_common(graph, name, operate, REFEREE_UI_GRAPH_FLOAT, layer, color);
    graph->start_angle = font_size;
    graph->end_angle = decimal_digits;
    graph->width = width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    set_number(graph, value);
}

void referee_ui_integer(referee_ui_graph_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                        uint32_t font_size, uint32_t width, uint32_t start_x, uint32_t start_y, int32_t value)
{
    set_common(graph, name, operate, REFEREE_UI_GRAPH_INT, layer, color);
    graph->start_angle = font_size;
    graph->width = width;
    graph->start_x = start_x;
    graph->start_y = start_y;
    set_number(graph, value);
}

void referee_ui_string(referee_ui_string_t *graph, const char name[3], uint32_t operate, uint32_t layer, uint32_t color,
                       uint32_t font_size, uint32_t width, uint32_t start_x, uint32_t start_y, const char *fmt, ...)
{
    char text[31];
    va_list arguments;

    memset(graph, 0, sizeof *graph);
    set_name(graph->graph.graphic_name, name);
    graph->graph.operate_type = operate;
    graph->graph.graphic_type = REFEREE_UI_GRAPH_STRING;
    graph->graph.layer = layer;
    graph->graph.color = color;
    graph->graph.width = width;
    graph->graph.start_x = start_x;
    graph->graph.start_y = start_y;
    graph->graph.start_angle = font_size;

    va_start(arguments, fmt);
    int written = vsnprintf(text, sizeof text, (fmt != NULL) ? fmt : "", arguments);

    va_end(arguments);
    if (written < 0)
    {
        written = 0;
    }
    if (written > 30)
    {
        written = 30;
    }
    memcpy(graph->text, text, (size_t)written);
    graph->graph.end_angle = (uint32_t)written;     /* 字符个数，选手端按这个长度显示 */
}

static int32_t send_to_client(uint16_t data_command_id, const void *content, uint16_t content_length)
{
    referee_id_t id;

    if (!referee_get_id(&id))
    {
        return ERR_BUSY;
    }
    return referee_send_interaction(data_command_id, id.client_id, content, content_length);
}

int32_t referee_ui_delete(uint8_t operate, uint8_t layer)
{
    uint8_t content[REFEREE_UI_DELETE_LENGTH] = { operate, layer };

    return send_to_client(REFEREE_INTERACTION_UI_DELETE, content, sizeof content);
}

int32_t referee_ui_graph_refresh(const referee_ui_graph_t *graphs, uint32_t count)
{
    uint16_t data_command_id;

    if (graphs == NULL)
    {
        return ERR_INVAL;
    }
    switch (count)
    {
        case 1u: data_command_id = REFEREE_INTERACTION_UI_DRAW_1; break;
        case 2u: data_command_id = REFEREE_INTERACTION_UI_DRAW_2; break;
        case 5u: data_command_id = REFEREE_INTERACTION_UI_DRAW_5; break;
        case 7u: data_command_id = REFEREE_INTERACTION_UI_DRAW_7; break;
        default: return ERR_INVAL;
    }
    return send_to_client(data_command_id, graphs, (uint16_t)(count * sizeof(referee_ui_graph_t)));
}

int32_t referee_ui_string_refresh(const referee_ui_string_t *graph)
{
    if (graph == NULL)
    {
        return ERR_INVAL;
    }
    return send_to_client(REFEREE_INTERACTION_UI_DRAW_STRING, graph, sizeof *graph);
}
