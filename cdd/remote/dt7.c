#include <string.h>
#include "cdd/remote/dt7.h"
#include "bsp/uart/bsp_uart.h"
#include "osal/event.h"
#include "component/common/initcall.h"
#include "component/common/err.h"

#define DT7_FRAME_LEN   18u
#define DT7_CH_MIN      364
#define DT7_CH_MID      1024
#define DT7_CH_MAX      1684
#define DT7_TIMEOUT_MS  100u            /* 超过这么久没收到合法帧 = 离线 */

static dt7_data_t cur;                  /* 只在 dt7_task 里写，别的任务用 dt7_get() 取快照 */

static bool ch_ok(int v) { return v >= DT7_CH_MIN && v <= DT7_CH_MAX; }

bool dt7_parse(const uint8_t *p, dt7_data_t *o)
{
    int c0 = ((p[0] | (p[1] << 8)) & 0x07FF);
    int c1 = (((p[1] >> 3) | (p[2] << 5)) & 0x07FF);
    int c2 = (((p[2] >> 6) | (p[3] << 2) | (p[4] << 10)) & 0x07FF);
    int c3 = (((p[4] >> 1) | (p[5] << 7)) & 0x07FF);
    int wh = ((p[16] | (p[17] << 8)) & 0x07FF);
    uint8_t s1 = (uint8_t)((p[5] >> 4) & 0x03), s2 = (uint8_t)((p[5] >> 6) & 0x03);

    /* 合法性：通道在 364..1684，开关只能是 1/2/3；DBUS 靠奇偶校验 + 这里的范围检查拦住干扰帧 */
    if (!ch_ok(c0) || !ch_ok(c1) || !ch_ok(c2) || !ch_ok(c3) || !ch_ok(wh) ||
        s1 == 0u || s2 == 0u) { return false; }

    o->ch[0] = (int16_t)(c0 - DT7_CH_MID);  o->ch[1] = (int16_t)(c1 - DT7_CH_MID);
    o->ch[2] = (int16_t)(c2 - DT7_CH_MID);  o->ch[3] = (int16_t)(c3 - DT7_CH_MID);
    o->wheel = (int16_t)(wh - DT7_CH_MID);
    o->s1 = s1; o->s2 = s2;
    o->mouse_x = (int16_t)(p[6]  | (p[7]  << 8));
    o->mouse_y = (int16_t)(p[8]  | (p[9]  << 8));
    o->mouse_z = (int16_t)(p[10] | (p[11] << 8));
    o->mouse_l = p[12]; o->mouse_r = p[13];
    o->key = (uint16_t)(p[14] | (p[15] << 8));
    return true;
}

/* 事件任务：UART5 每次空闲线收到的一帧（≤ 36 字节）由中断拷进邮箱，这里解析。
 *   EVENT_LATEST + 深度 1：只保留最新一帧，任务慢了旧帧被覆盖，不会处理过期数据
 *   超时 100 ms：data == NULL，视为遥控器断连——把全部控制量清零、online 置 false；
 *   之后再收到合法帧，整份数据被新帧覆盖，online 恢复为 true */
static void dt7_run(void *arg, const uint8_t *data, uint16_t len)
{
    (void)arg;
    if (data == NULL) {                                    /* 超时：断连，控制量清零 */
        uint32_t key = os_irq_save();
        uint32_t frames = cur.frames, bad = cur.bad_frames;
        memset(&cur, 0, sizeof cur);                       /* 通道、拨轮、鼠标、键盘、开关(0=未知)、online 全部清零 */
        cur.frames = frames;
        cur.bad_frames = bad;
        os_irq_restore(key);
        return;
    }
    dt7_data_t tmp = cur;                                  /* cur 只在本任务里写，这里读不用加锁 */
    if (len == DT7_FRAME_LEN && dt7_parse(data, &tmp)) {
        tmp.online = true;                                 /* 收到合法帧：恢复在线 */
        tmp.frames++;
        uint32_t key = os_irq_save();
        cur = tmp;
        os_irq_restore(key);
    } else {
        cur.bad_frames++;
    }
}
EVENT_TASK_DEFINE(dt7_task, 3, 384, EVENT_LATEST, 36, 1, DT7_TIMEOUT_MS, dt7_run, NULL);

void dt7_get(dt7_data_t *out)
{
    uint32_t key = os_irq_save();
    *out = cur;
    os_irq_restore(key);
}

static int32_t dt7_init(void)
{
    memset(&cur, 0, sizeof cur);
    bsp_uart_bind_event(UART_DBUS, &dt7_task);
    return ERR_OK;
}
INIT_CDD(dt7_init);
