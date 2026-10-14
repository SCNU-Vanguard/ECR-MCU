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
static uint8_t      motor_count;
static bool         subscribed[CAN_NUM];

static uint32_t feedback_id(const dji_motor_t *motor)
{
    return (motor->type == DJI_GM6020 ? 0x204u : 0x200u) + motor->id;
}

/* 控制帧：返回 tx_ids 下标，*slot = 帧内位置 0..3；非法返回 -1 */
static int tx_slot(dji_type_t type, uint8_t id, uint8_t *slot)
{
    if (id == 0u)
    {
        return -1;
    }
    if (type == DJI_GM6020)
    {
        if (id > 7u)
        {
            return -1;
        }
        *slot = (uint8_t)((id - 1u) & 3u);
        return (id <= 4u) ? 1 : 2;
    }
    if (id > 8u)
    {
        return -1;
    }
    *slot = (uint8_t)((id - 1u) & 3u);
    return (id <= 4u) ? 0 : 1;
}

/* 事件任务：CAN 中断把 0x201..0x20B 的帧拷成 can_event_t 投递过来，这里解码。
 * 优先级设为 0（最高）：先于 1 kHz 控制任务更新反馈，控制任务读到的就是最新的 */
static void dji_run(void *argument, const uint8_t *message, uint16_t len)
{
    can_event_t event;

    (void)argument;
    if (message == NULL || len != sizeof event)
    {
        return;
    }
    memcpy(&event, message, sizeof event);
    if (event.len < 7u)
    {
        return;
    }
    for (uint8_t motor_index = 0; motor_index < motor_count; motor_index++)
    {
        dji_motor_t *motor = motors[motor_index];

        if (motor->bus != (can_id_t)event.bus || feedback_id(motor) != event.id)
        {
            continue;
        }
        uint16_t angle = (uint16_t)((event.data[0] << 8) | event.data[1]);
        uint32_t irq_key = os_irq_save();                   /* 读者在别的任务里，几个字段要一起更新 */

        if (!motor->got_first)
        {
            motor->got_first = true;
        }
        else
        {
            int angle_delta = (int)angle - (int)motor->last_angle;

            if (angle_delta > 4096)
            {
                motor->round_count--;                       /* 反转过零点 */
            }
            else if (angle_delta < -4096)
            {
                motor->round_count++;                       /* 正转过零点 */
            }
        }
        motor->last_angle  = angle;
        motor->angle       = angle;
        motor->total_angle = motor->round_count * 8192 + (int32_t)angle;
        motor->speed_rpm   = (int16_t)((event.data[2] << 8) | event.data[3]);
        motor->current     = (int16_t)((event.data[4] << 8) | event.data[5]);
        motor->temperature = event.data[6];
        motor->rx_ms       = bsp_tick_ms();
        os_irq_restore(irq_key);
        return;
    }
}
EVENT_TASK_DEFINE(dji_task, 0, 384, EVENT_QUEUE, sizeof(can_event_t), 32, 0, dji_run, NULL);

int32_t dji_motor_register(dji_motor_t *motor, can_id_t bus, uint8_t id, dji_type_t type)
{
    uint8_t slot;

    if (motor == NULL || (unsigned)bus >= CAN_NUM || tx_slot(type, id, &slot) < 0)
    {
        return ERR_INVAL;
    }
    if (motor_count >= DJI_MAX_MOTORS)
    {
        return ERR_NOMEM;
    }

    memset(motor, 0, sizeof *motor);
    motor->bus = bus;
    motor->id = id;
    motor->type = type;
    for (uint8_t motor_index = 0; motor_index < motor_count; motor_index++)    /* 同总线、反馈ID相同 = 冲突 */
    {
        if (motors[motor_index]->bus == bus && feedback_id(motors[motor_index]) == feedback_id(motor))
        {
            return ERR_BUSY;
        }
    }
    if (!subscribed[bus])
    {
        int32_t subscribe_ret = bsp_can_subscribe_event(bus, FB_ID_MIN, FB_ID_MAX, &dji_task);

        if (subscribe_ret != ERR_OK)
        {
            return subscribe_ret;
        }
        subscribed[bus] = true;
    }
    motors[motor_count] = motor;
    motor_count++;
    return ERR_OK;
}

void dji_motor_set_cmd(dji_motor_t *motor, int16_t cmd)
{
    motor->cmd = cmd;
}

bool dji_motor_online(const dji_motor_t *motor, uint32_t timeout_ms)
{
    return motor->got_first && (bsp_tick_ms() - motor->rx_ms) < timeout_ms;
}

void dji_motor_zero(dji_motor_t *motor)
{
    uint32_t irq_key = os_irq_save();

    motor->round_count = 0;
    motor->total_angle = 0;
    motor->got_first = false;           /* 下一帧重新作为起点 */
    os_irq_restore(irq_key);
}

int32_t dji_motor_send_all(void)
{
    int16_t values[CAN_NUM][3][4];
    bool    used[CAN_NUM][3];

    memset(values, 0, sizeof values);
    memset(used, 0, sizeof used);

    for (uint8_t motor_index = 0; motor_index < motor_count; motor_index++)
    {
        uint8_t slot = 0;
        int frame_index = tx_slot(motors[motor_index]->type, motors[motor_index]->id, &slot);

        if (frame_index < 0)                                /* register 时已校验，这里只是防御 */
        {
            continue;
        }
        values[motors[motor_index]->bus][frame_index][slot] = motors[motor_index]->cmd;
        used[motors[motor_index]->bus][frame_index] = true;
    }

    int32_t sent_count = 0;

    for (int bus = 0; bus < CAN_NUM; bus++)
    {
        for (int frame_index = 0; frame_index < 3; frame_index++)
        {
            if (!used[bus][frame_index])                    /* 没有电机的帧不发：无应答的帧会被无限重发 */
            {
                continue;
            }
            can_frame_t frame;

            frame.id = tx_ids[frame_index];
            frame.len = 8;
            for (int slot = 0; slot < 4; slot++)
            {
                frame.data[2 * slot]     = (uint8_t)((uint16_t)values[bus][frame_index][slot] >> 8);
                frame.data[2 * slot + 1] = (uint8_t)(values[bus][frame_index][slot] & 0xFF);
            }
            if (bsp_can_send((can_id_t)bus, &frame) == ERR_OK)
            {
                sent_count++;
            }
        }
    }
    return sent_count;
}
