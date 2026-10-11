#include <string.h>
#include "cdd/motor/dji_motor/dji_motor.h"
#include "bsp/can/bsp_can.h"
#include "bsp/tick/bsp_tick.h"
#include "component/common/err.h"
#include "osal/osal.h"
#include "osal/event.h"

/* DJI 协议：
 *   反馈帧 ID：M3508/M2006 = 0x200 + id；GM6020 = 0x204 + id；8 字节，大端：
 *              转子角度(2) 转速rpm(2) 电流/转矩(2) 温度(1) 保留(1)
 *   控制帧：   4 个 int16 大端，对应 4 个电机；
 *              M3508/M2006：id1..4 -> 0x200，id5..8 -> 0x1FF
 *              GM6020(电压)：id1..4 -> 0x1FF，id5..7 -> 0x2FF
 * 注意：同一条总线上 M3508 的 id5..8 与 GM6020 的 id1..4 反馈 ID/控制帧 ID 重叠，不能混接。 */
#define FB_ID_MIN   0x201u
#define FB_ID_MAX   0x20Bu

static const uint16_t tx_ids[3] = { 0x200, 0x1FF, 0x2FF };

static dji_motor_t *motors[DJI_MAX_MOTORS];
static uint8_t      nmotors;
static bool         subscribed[CAN_NUM];

static uint32_t fb_id(const dji_motor_t *m)
{
    return (m->type == DJI_GM6020 ? 0x204u : 0x200u) + m->id;
}

/* 控制帧：返回 tx_ids 下标，*slot = 帧内位置 0..3；非法返回 -1 */
static int tx_slot(dji_type_t type, uint8_t id, uint8_t *slot)
{
    if (id == 0u) { return -1; }
    if (type == DJI_GM6020) {
        if (id > 7u) { return -1; }
        *slot = (uint8_t)((id - 1u) & 3u);
        return (id <= 4u) ? 1 : 2;
    }
    if (id > 8u) { return -1; }
    *slot = (uint8_t)((id - 1u) & 3u);
    return (id <= 4u) ? 0 : 1;
}

/* 事件任务：CAN 中断把 0x201..0x20B 的帧拷成 can_event_t 投递过来，这里解码。
 * 优先级设为 0（最高）：先于 1 kHz 控制任务更新反馈，控制任务读到的就是最新的 */
static void dji_run(void *arg, const uint8_t *msg, uint16_t len)
{
    (void)arg;
    can_event_t e;
    if (msg == NULL || len != sizeof e) { return; }
    memcpy(&e, msg, sizeof e);
    if (e.len < 7u) { return; }
    for (uint8_t i = 0; i < nmotors; i++) {
        dji_motor_t *m = motors[i];
        if (m->bus != (can_id_t)e.bus || fb_id(m) != e.id) { continue; }
        uint16_t ang = (uint16_t)((e.data[0] << 8) | e.data[1]);
        uint32_t key = os_irq_save();                       /* 读者在别的任务里，几个字段要一起更新 */
        if (!m->got_first) {
            m->got_first = true;
        } else {
            int d = (int)ang - (int)m->last_angle;
            if (d > 4096)       { m->round_cnt--; }         /* 反转过零点 */
            else if (d < -4096) { m->round_cnt++; }         /* 正转过零点 */
        }
        m->last_angle  = ang;
        m->angle       = ang;
        m->total_angle = m->round_cnt * 8192 + (int32_t)ang;
        m->speed_rpm   = (int16_t)((e.data[2] << 8) | e.data[3]);
        m->current     = (int16_t)((e.data[4] << 8) | e.data[5]);
        m->temperature = e.data[6];
        m->rx_ms       = bsp_tick_ms();
        os_irq_restore(key);
        return;
    }
}
EVENT_TASK_DEFINE(dji_task, 0, 384, EVENT_QUEUE, sizeof(can_event_t), 32, 0, dji_run, NULL);

int32_t dji_motor_register(dji_motor_t *m, can_id_t bus, uint8_t id, dji_type_t type)
{
    uint8_t slot;
    if (m == NULL || (unsigned)bus >= CAN_NUM || tx_slot(type, id, &slot) < 0) { return ERR_INVAL; }
    if (nmotors >= DJI_MAX_MOTORS) { return ERR_NOMEM; }

    memset(m, 0, sizeof *m);
    m->bus = bus; m->id = id; m->type = type;
    for (uint8_t i = 0; i < nmotors; i++) {                 /* 同总线、反馈ID相同 = 冲突 */
        if (motors[i]->bus == bus && fb_id(motors[i]) == fb_id(m)) { return ERR_BUSY; }
    }
    if (!subscribed[bus]) {
        int32_t r = bsp_can_subscribe_event(bus, FB_ID_MIN, FB_ID_MAX, &dji_task);
        if (r != ERR_OK) { return r; }
        subscribed[bus] = true;
    }
    motors[nmotors++] = m;
    return ERR_OK;
}

void dji_motor_set_cmd(dji_motor_t *m, int16_t cmd) { m->cmd = cmd; }

bool dji_motor_online(const dji_motor_t *m, uint32_t timeout_ms)
{
    return m->got_first && (bsp_tick_ms() - m->rx_ms) < timeout_ms;
}

void dji_motor_zero(dji_motor_t *m)
{
    uint32_t key = os_irq_save();
    m->round_cnt = 0;
    m->total_angle = 0;
    m->got_first = false;           /* 下一帧重新作为起点 */
    os_irq_restore(key);
}

int32_t dji_motor_send_all(void)
{
    int16_t val[CAN_NUM][3][4];
    bool    used[CAN_NUM][3];
    memset(val, 0, sizeof val);
    memset(used, 0, sizeof used);

    for (uint8_t i = 0; i < nmotors; i++) {
        uint8_t slot = 0;
        int t = tx_slot(motors[i]->type, motors[i]->id, &slot);
        if (t < 0) { continue; }                            /* register 时已校验，这里只是防御 */
        val[motors[i]->bus][t][slot] = motors[i]->cmd;
        used[motors[i]->bus][t] = true;
    }

    int32_t sent = 0;
    for (int b = 0; b < CAN_NUM; b++) {
        for (int t = 0; t < 3; t++) {
            if (!used[b][t]) { continue; }                  /* 没有电机的帧不发：无应答的帧会被无限重发 */
            can_frame_t f = { .id = tx_ids[t], .len = 8 };
            for (int k = 0; k < 4; k++) {
                f.data[2 * k]     = (uint8_t)((uint16_t)val[b][t][k] >> 8);
                f.data[2 * k + 1] = (uint8_t)(val[b][t][k] & 0xFF);
            }
            if (bsp_can_send((can_id_t)b, &f) == ERR_OK) { sent++; }
        }
    }
    return sent;
}
