#include <string.h>
#include "cdd/referee/ui/ui_reg.h"
#include "component/common/err.h"
#include "osal/period.h"

/* ui_config.h 展开成图形对象，默认值直接写进协议结构。
 * 10 Hz 任务每拍最多发一包。还没 ADD 的图形优先，然后是字符串，再是改过的图形。
 * 图形如果每拍都在变，脏字符串隔一拍插入。先清 dirty 再拷贝，拷贝之后的修改会重新置脏。 */

#define UI_LAYER_MAX   15u
#define UI_COLOR_MAX   15u
#define UI_WIDTH_MAX   1023u
#define UI_COORD_MAX   2047u
#define UI_RADIUS_MAX  1023u
#define UI_ANGLE_MAX   511u

#define UI_BIT(_type)    (1u << (_type))
#define UI_BIT_ALL       0xFFu
#define UI_BIT_END       (UI_BIT(REFEREE_UI_GRAPH_LINE) | UI_BIT(REFEREE_UI_GRAPH_RECTANGLE) | \
                          UI_BIT(REFEREE_UI_GRAPH_ELLIPSE) | UI_BIT(REFEREE_UI_GRAPH_ARC))
#define UI_BIT_FONT      (UI_BIT(REFEREE_UI_GRAPH_FLOAT) | UI_BIT(REFEREE_UI_GRAPH_INT) | UI_BIT(REFEREE_UI_GRAPH_STRING))

struct ui_figure
{
    referee_ui_string_t packed; /* 非字符串不用 text */
    uint8_t             type;
    volatile bool       added;
    volatile bool       dirty;
};

/* 协议名第一个字符放在 graphic_name[2]，和原 UI 库一致。 */
#define UI_COMMON(_proto, _type, _layer, _color) \
    .packed.graph.graphic_name = { (uint8_t)(_proto)[2], (uint8_t)(_proto)[1], (uint8_t)(_proto)[0] }, \
    .packed.graph.operate_type = REFEREE_UI_OPERATE_ADD, \
    .packed.graph.graphic_type = (_type), \
    .packed.graph.layer = (_layer), \
    .packed.graph.color = (_color), \
    .type = (uint8_t)(_type), \
    .dirty = true

#define UI_CHECK(_value, _max) \
    _Static_assert((uint32_t)(_value) <= (uint32_t)(_max), "ui 字段超出协议位宽")
#define UI_CHECK_NAME(_proto) \
    _Static_assert(sizeof(_proto) == 4, "ui 名字必须是 3 个字符")
#define UI_CHECK_XY(_x, _y) \
    UI_CHECK(_x, UI_COORD_MAX); \
    UI_CHECK(_y, UI_COORD_MAX)

#define UI_LINE(_name, _proto, _layer, _color, _width, _x0, _y0, _x1, _y1) \
    UI_CHECK_NAME(_proto); \
    UI_CHECK(_layer, UI_LAYER_MAX); \
    UI_CHECK(_color, UI_COLOR_MAX); \
    UI_CHECK(_width, UI_WIDTH_MAX); \
    UI_CHECK_XY(_x0, _y0); \
    UI_CHECK_XY(_x1, _y1); \
    ui_figure_t _name = { \
        UI_COMMON(_proto, REFEREE_UI_GRAPH_LINE, _layer, _color), \
        .packed.graph.width = (_width), \
        .packed.graph.start_x = (_x0), \
        .packed.graph.start_y = (_y0), \
        .packed.graph.end_x = (_x1), \
        .packed.graph.end_y = (_y1), \
    };

#define UI_RECT(_name, _proto, _layer, _color, _width, _x0, _y0, _x1, _y1) \
    UI_CHECK_NAME(_proto); \
    UI_CHECK(_layer, UI_LAYER_MAX); \
    UI_CHECK(_color, UI_COLOR_MAX); \
    UI_CHECK(_width, UI_WIDTH_MAX); \
    UI_CHECK_XY(_x0, _y0); \
    UI_CHECK_XY(_x1, _y1); \
    ui_figure_t _name = { \
        UI_COMMON(_proto, REFEREE_UI_GRAPH_RECTANGLE, _layer, _color), \
        .packed.graph.width = (_width), \
        .packed.graph.start_x = (_x0), \
        .packed.graph.start_y = (_y0), \
        .packed.graph.end_x = (_x1), \
        .packed.graph.end_y = (_y1), \
    };

#define UI_CIRCLE(_name, _proto, _layer, _color, _width, _x, _y, _radius) \
    UI_CHECK_NAME(_proto); \
    UI_CHECK(_layer, UI_LAYER_MAX); \
    UI_CHECK(_color, UI_COLOR_MAX); \
    UI_CHECK(_width, UI_WIDTH_MAX); \
    UI_CHECK_XY(_x, _y); \
    UI_CHECK(_radius, UI_RADIUS_MAX); \
    ui_figure_t _name = { \
        UI_COMMON(_proto, REFEREE_UI_GRAPH_CIRCLE, _layer, _color), \
        .packed.graph.width = (_width), \
        .packed.graph.start_x = (_x), \
        .packed.graph.start_y = (_y), \
        .packed.graph.radius = (_radius), \
    };

#define UI_ELLIPSE(_name, _proto, _layer, _color, _width, _x, _y, _a, _b) \
    UI_CHECK_NAME(_proto); \
    UI_CHECK(_layer, UI_LAYER_MAX); \
    UI_CHECK(_color, UI_COLOR_MAX); \
    UI_CHECK(_width, UI_WIDTH_MAX); \
    UI_CHECK_XY(_x, _y); \
    UI_CHECK_XY(_a, _b); \
    ui_figure_t _name = { \
        UI_COMMON(_proto, REFEREE_UI_GRAPH_ELLIPSE, _layer, _color), \
        .packed.graph.width = (_width), \
        .packed.graph.start_x = (_x), \
        .packed.graph.start_y = (_y), \
        .packed.graph.end_x = (_a), \
        .packed.graph.end_y = (_b), \
    };

#define UI_ARC(_name, _proto, _layer, _color, _start_deg, _end_deg, _width, _x, _y, _a, _b) \
    UI_CHECK_NAME(_proto); \
    UI_CHECK(_layer, UI_LAYER_MAX); \
    UI_CHECK(_color, UI_COLOR_MAX); \
    UI_CHECK(_width, UI_WIDTH_MAX); \
    UI_CHECK(_start_deg, UI_ANGLE_MAX); \
    UI_CHECK(_end_deg, UI_ANGLE_MAX); \
    UI_CHECK_XY(_x, _y); \
    UI_CHECK_XY(_a, _b); \
    ui_figure_t _name = { \
        UI_COMMON(_proto, REFEREE_UI_GRAPH_ARC, _layer, _color), \
        .packed.graph.start_angle = (_start_deg), \
        .packed.graph.end_angle = (_end_deg), \
        .packed.graph.width = (_width), \
        .packed.graph.start_x = (_x), \
        .packed.graph.start_y = (_y), \
        .packed.graph.end_x = (_a), \
        .packed.graph.end_y = (_b), \
    };

#define UI_FLOAT(_name, _proto, _layer, _color, _font, _digits, _width, _x, _y, _value) \
    UI_CHECK_NAME(_proto); \
    UI_CHECK(_layer, UI_LAYER_MAX); \
    UI_CHECK(_color, UI_COLOR_MAX); \
    UI_CHECK(_width, UI_WIDTH_MAX); \
    UI_CHECK(_font, UI_ANGLE_MAX); \
    UI_CHECK(_digits, UI_ANGLE_MAX); \
    UI_CHECK_XY(_x, _y); \
    ui_figure_t _name = { \
        UI_COMMON(_proto, REFEREE_UI_GRAPH_FLOAT, _layer, _color), \
        .packed.graph.start_angle = (_font), \
        .packed.graph.end_angle = (_digits), \
        .packed.graph.width = (_width), \
        .packed.graph.start_x = (_x), \
        .packed.graph.start_y = (_y), \
        .packed.graph.radius = REFEREE_UI_VALUE_RADIUS(_value), \
        .packed.graph.end_x = REFEREE_UI_VALUE_END_X(_value), \
        .packed.graph.end_y = REFEREE_UI_VALUE_END_Y(_value), \
    };

#define UI_INT(_name, _proto, _layer, _color, _font, _width, _x, _y, _value) \
    UI_CHECK_NAME(_proto); \
    UI_CHECK(_layer, UI_LAYER_MAX); \
    UI_CHECK(_color, UI_COLOR_MAX); \
    UI_CHECK(_width, UI_WIDTH_MAX); \
    UI_CHECK(_font, UI_ANGLE_MAX); \
    UI_CHECK_XY(_x, _y); \
    ui_figure_t _name = { \
        UI_COMMON(_proto, REFEREE_UI_GRAPH_INT, _layer, _color), \
        .packed.graph.start_angle = (_font), \
        .packed.graph.width = (_width), \
        .packed.graph.start_x = (_x), \
        .packed.graph.start_y = (_y), \
        .packed.graph.radius = REFEREE_UI_VALUE_RADIUS(_value), \
        .packed.graph.end_x = REFEREE_UI_VALUE_END_X(_value), \
        .packed.graph.end_y = REFEREE_UI_VALUE_END_Y(_value), \
    };

#define UI_STR(_name, _proto, _layer, _color, _font, _width, _x, _y, _text) \
    UI_CHECK_NAME(_proto); \
    UI_CHECK(_layer, UI_LAYER_MAX); \
    UI_CHECK(_color, UI_COLOR_MAX); \
    UI_CHECK(_width, UI_WIDTH_MAX); \
    UI_CHECK(_font, UI_ANGLE_MAX); \
    UI_CHECK_XY(_x, _y); \
    _Static_assert(sizeof(_text) <= 31, "ui 字符串超过 30 个字符"); \
    ui_figure_t _name = { \
        UI_COMMON(_proto, REFEREE_UI_GRAPH_STRING, _layer, _color), \
        .packed.graph.start_angle = (_font), \
        .packed.graph.end_angle = sizeof(_text) - 1u, \
        .packed.graph.width = (_width), \
        .packed.graph.start_x = (_x), \
        .packed.graph.start_y = (_y), \
        .packed.text = _text, \
    };

ROBOT_UI_FIGURE_TABLE

#undef UI_LINE
#undef UI_RECT
#undef UI_CIRCLE
#undef UI_ELLIPSE
#undef UI_ARC
#undef UI_FLOAT
#undef UI_INT
#undef UI_STR
#undef UI_COMMON
#undef UI_CHECK
#undef UI_CHECK_NAME
#undef UI_CHECK_XY

static ui_figure_t *const ui_figures[] = {
#define UI_LINE(_name, ...)    &_name,
#define UI_RECT(_name, ...)    &_name,
#define UI_CIRCLE(_name, ...)  &_name,
#define UI_ELLIPSE(_name, ...) &_name,
#define UI_ARC(_name, ...)     &_name,
#define UI_FLOAT(_name, ...)   &_name,
#define UI_INT(_name, ...)     &_name,
#define UI_STR(_name, ...)     &_name,
    ROBOT_UI_FIGURE_TABLE
#undef UI_LINE
#undef UI_RECT
#undef UI_CIRCLE
#undef UI_ELLIPSE
#undef UI_ARC
#undef UI_FLOAT
#undef UI_INT
#undef UI_STR
    (ui_figure_t *)0
};

static const uint32_t ui_figure_count = (uint32_t)((sizeof ui_figures / sizeof ui_figures[0]) - 1u);
static volatile uint8_t redraw_request;
static bool string_next;

static void touch(ui_figure_t *fig)
{
    fig->dirty = true;
}

static bool fig_ok(const ui_figure_t *fig, uint32_t type_mask)
{
    return fig != NULL && (type_mask & UI_BIT(fig->type)) != 0u;
}

void ui_set_layer(ui_figure_t *fig, uint32_t layer)
{
    if (!fig_ok(fig, UI_BIT_ALL) || fig->packed.graph.layer == layer)
    {
        return;
    }
    fig->packed.graph.layer = layer;
    touch(fig);
}

void ui_set_color(ui_figure_t *fig, uint32_t color)
{
    if (!fig_ok(fig, UI_BIT_ALL) || fig->packed.graph.color == color)
    {
        return;
    }
    fig->packed.graph.color = color;
    touch(fig);
}

void ui_set_width(ui_figure_t *fig, uint32_t width)
{
    if (!fig_ok(fig, UI_BIT_ALL) || fig->packed.graph.width == width)
    {
        return;
    }
    fig->packed.graph.width = width;
    touch(fig);
}

void ui_set_start(ui_figure_t *fig, uint32_t x, uint32_t y)
{
    if (!fig_ok(fig, UI_BIT_ALL) || (fig->packed.graph.start_x == x && fig->packed.graph.start_y == y))
    {
        return;
    }
    fig->packed.graph.start_x = x;
    fig->packed.graph.start_y = y;
    touch(fig);
}

void ui_set_end(ui_figure_t *fig, uint32_t x, uint32_t y)
{
    if (!fig_ok(fig, UI_BIT_END) || (fig->packed.graph.end_x == x && fig->packed.graph.end_y == y))
    {
        return;
    }
    fig->packed.graph.end_x = x;
    fig->packed.graph.end_y = y;
    touch(fig);
}

void ui_set_radius(ui_figure_t *fig, uint32_t radius)
{
    if (!fig_ok(fig, UI_BIT(REFEREE_UI_GRAPH_CIRCLE)) || fig->packed.graph.radius == radius)
    {
        return;
    }
    fig->packed.graph.radius = radius;
    touch(fig);
}

void ui_set_angle(ui_figure_t *fig, uint32_t start_deg, uint32_t end_deg)
{
    if (!fig_ok(fig, UI_BIT(REFEREE_UI_GRAPH_ARC))
        || (fig->packed.graph.start_angle == start_deg && fig->packed.graph.end_angle == end_deg))
    {
        return;
    }
    fig->packed.graph.start_angle = start_deg;
    fig->packed.graph.end_angle = end_deg;
    touch(fig);
}

void ui_set_font(ui_figure_t *fig, uint32_t font_size)
{
    if (!fig_ok(fig, UI_BIT_FONT) || fig->packed.graph.start_angle == font_size)
    {
        return;
    }
    fig->packed.graph.start_angle = font_size;
    touch(fig);
}

void ui_set_decimals(ui_figure_t *fig, uint32_t digits)
{
    if (!fig_ok(fig, UI_BIT(REFEREE_UI_GRAPH_FLOAT)) || fig->packed.graph.end_angle == digits)
    {
        return;
    }
    fig->packed.graph.end_angle = digits;
    touch(fig);
}

static void set_number(ui_figure_t *fig, uint32_t type_bit, int32_t value)
{
    uint32_t radius = REFEREE_UI_VALUE_RADIUS(value);
    uint32_t end_x = REFEREE_UI_VALUE_END_X(value);
    uint32_t end_y = REFEREE_UI_VALUE_END_Y(value);

    if (!fig_ok(fig, type_bit)
        || (fig->packed.graph.radius == radius && fig->packed.graph.end_x == end_x && fig->packed.graph.end_y == end_y))
    {
        return;
    }
    fig->packed.graph.radius = radius;
    fig->packed.graph.end_x = end_x;
    fig->packed.graph.end_y = end_y;
    touch(fig);
}

void ui_set_int(ui_figure_t *fig, int32_t value)
{
    set_number(fig, UI_BIT(REFEREE_UI_GRAPH_INT), value);
}

void ui_set_float(ui_figure_t *fig, int32_t value_x1000)
{
    set_number(fig, UI_BIT(REFEREE_UI_GRAPH_FLOAT), value_x1000);
}

void ui_set_string(ui_figure_t *fig, const char *text)
{
    uint8_t buf[30];
    uint32_t length = 0;

    if (!fig_ok(fig, UI_BIT(REFEREE_UI_GRAPH_STRING)))
    {
        return;
    }
    if (text != NULL)
    {
        while (length < sizeof buf && text[length] != '\0')
        {
            buf[length] = (uint8_t)text[length];
            length++;
        }
    }
    if (fig->packed.graph.end_angle == length && memcmp(fig->packed.text, buf, length) == 0)
    {
        return;
    }
    memset(fig->packed.text, 0, sizeof fig->packed.text);
    memcpy(fig->packed.text, buf, length);
    fig->packed.graph.end_angle = length;
    touch(fig);
}

void ui_redraw(void)
{
    redraw_request = 1u;
}

static bool waiting(const ui_figure_t *fig, bool as_string, bool unadded_only)
{
    if ((fig->type == REFEREE_UI_GRAPH_STRING) != as_string)
    {
        return false;
    }
    return unadded_only ? !fig->added : fig->dirty;
}

static ui_figure_t *first_waiting(bool as_string, bool unadded_only)
{
    uint32_t i;

    for (i = 0; i < ui_figure_count; i++)
    {
        if (waiting(ui_figures[i], as_string, unadded_only))
        {
            return ui_figures[i];
        }
    }
    return NULL;
}

static void prepare(ui_figure_t *fig)
{
    fig->dirty = false;
    fig->packed.graph.operate_type = fig->added ? REFEREE_UI_OPERATE_CHANGE : REFEREE_UI_OPERATE_ADD;
}

static void finish(ui_figure_t *fig, bool sent)
{
    if (sent)
    {
        fig->added = true;
        return;
    }
    fig->dirty = true;
}

static uint32_t batch_count(uint32_t count)
{
    if (count >= 7u)
    {
        return 7u;
    }
    if (count >= 5u)
    {
        return 5u;
    }
    if (count >= 2u)
    {
        return 2u;
    }
    return count;
}

static int32_t send_graphs(bool unadded_only)
{
    ui_figure_t *pick[7];
    referee_ui_graph_t batch[7];
    uint32_t found = 0;
    uint32_t count;
    uint32_t i;
    int32_t ret;

    for (i = 0; i < ui_figure_count && found < 7u; i++)
    {
        if (waiting(ui_figures[i], false, unadded_only))
        {
            pick[found++] = ui_figures[i];
        }
    }
    count = batch_count(found);
    if (count == 0u)
    {
        return ERR_OK;
    }
    for (i = 0; i < count; i++)
    {
        prepare(pick[i]);
        batch[i] = pick[i]->packed.graph;
    }
    ret = referee_ui_graph_refresh(batch, count);
    for (i = 0; i < count; i++)
    {
        finish(pick[i], ret == ERR_OK);
    }
    return ret;
}

static int32_t send_string(ui_figure_t *fig)
{
    int32_t ret;

    prepare(fig);
    ret = referee_ui_string_refresh(&fig->packed);
    finish(fig, ret == ERR_OK);
    return ret;
}

static void apply_redraw(void)
{
    uint32_t i;

    if (redraw_request == 0u)
    {
        return;
    }
    redraw_request = 0u;
    for (i = 0; i < ui_figure_count; i++)
    {
        ui_figures[i]->added = false;
        ui_figures[i]->dirty = true;
    }
}

static int32_t ui_flush(void)
{
    ui_figure_t *string;

    apply_redraw();
    if (first_waiting(false, true) != NULL)
    {
        return send_graphs(true);
    }

    string = first_waiting(true, true);
    if (string == NULL && string_next)
    {
        string = first_waiting(true, false);
    }
    if (string != NULL)
    {
        string_next = false;
        return send_string(string);
    }

    if (first_waiting(false, false) != NULL)
    {
        string_next = first_waiting(true, false) != NULL;
        return send_graphs(false);
    }
    string = first_waiting(true, false);
    if (string != NULL)
    {
        return send_string(string);
    }
    return ERR_OK;
}

static void ui_tx_run(void *argument)
{
    (void)argument;
    (void)ui_flush();
}
PERIOD_TASK_DEFINE(ui_tx_task, OS_FREQ_10HZ, OS_PRIORITY_LOW, OS_STACK_MEDIUM, ui_tx_run, NULL);
