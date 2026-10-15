#include <string.h>
#include "cdd/remote/dt7.h"
#include "bsp/uart/bsp_uart.h"
#include "osal/event.h"
#include "component/common/initcall.h"
#include "component/common/err.h"

#define DT7_FRAME_LEN   18u
#define DT7_CHANNEL_MIN      364
#define DT7_CHANNEL_MID      1024
#define DT7_CHANNEL_MAX      1684
#define DT7_TIMEOUT_MS  100u            /* 超过这么久没收到合法帧 = 离线 */

static dt7_data_t current_data;         /* 只在 dt7_task 里写，别的任务用 dt7_get() 取快照 */

static bool channel_valid(int value)
{
    return value >= DT7_CHANNEL_MIN && value <= DT7_CHANNEL_MAX;
}

bool dt7_parse(const uint8_t *frame, dt7_data_t *out)
{
    int channel0 = ((frame[0] | (frame[1] << 8)) & 0x07FF);
    int channel1 = (((frame[1] >> 3) | (frame[2] << 5)) & 0x07FF);
    int channel2 = (((frame[2] >> 6) | (frame[3] << 2) | (frame[4] << 10)) & 0x07FF);
    int channel3 = (((frame[4] >> 1) | (frame[5] << 7)) & 0x07FF);
    int wheel = ((frame[16] | (frame[17] << 8)) & 0x07FF);
    uint8_t switch_left = (uint8_t)((frame[5] >> 4) & 0x03);
    uint8_t switch_right = (uint8_t)((frame[5] >> 6) & 0x03);

    /* 合法性：通道在 364..1684，开关只能是 1/2/3；DBUS 靠奇偶校验 + 这里的范围检查拦住干扰帧 */
    if (!channel_valid(channel0) || !channel_valid(channel1) || !channel_valid(channel2) ||
        !channel_valid(channel3) || !channel_valid(wheel) ||
        switch_left == 0u || switch_right == 0u)
    {
        return false;
    }

    out->channel[0] = (int16_t)(channel0 - DT7_CHANNEL_MID);
    out->channel[1] = (int16_t)(channel1 - DT7_CHANNEL_MID);
    out->channel[2] = (int16_t)(channel2 - DT7_CHANNEL_MID);
    out->channel[3] = (int16_t)(channel3 - DT7_CHANNEL_MID);
    out->wheel = (int16_t)(wheel - DT7_CHANNEL_MID);
    out->switch_left = switch_left;
    out->switch_right = switch_right;
    out->mouse_x = (int16_t)(frame[6]  | (frame[7]  << 8));
    out->mouse_y = (int16_t)(frame[8]  | (frame[9]  << 8));
    out->mouse_z = (int16_t)(frame[10] | (frame[11] << 8));
    out->mouse_left = frame[12];
    out->mouse_right = frame[13];
    out->key = (uint16_t)(frame[14] | (frame[15] << 8));
    return true;
}

/* 事件任务：UART5 每次空闲线收到的一帧（≤ 36 字节）由中断拷进邮箱，这里解析。
 *   EVENT_LATEST + 深度 1：只保留最新一帧，任务慢了旧帧被覆盖，不会处理过期数据
 *   超时 100 ms：data == NULL，视为遥控器断连——把全部控制量清零、online 置 false；
 *   之后再收到合法帧，整份数据被新帧覆盖，online 恢复为 true */
static void dt7_run(void *argument, const uint8_t *data, uint16_t len)
{
    (void)argument;
    if (data == NULL)                                       /* 超时：断连，控制量清零 */
    {
        uint32_t irq_key = os_irq_save();
        uint32_t frames = current_data.frames;
        uint32_t bad_frames = current_data.bad_frames;

        memset(&current_data, 0, sizeof current_data);      /* 通道、拨轮、鼠标、键盘、开关(0=未知)、online 全部清零 */
        current_data.frames = frames;
        current_data.bad_frames = bad_frames;
        os_irq_restore(irq_key);
        return;
    }
    dt7_data_t new_data = current_data;                     /* current_data 只在本任务里写，这里读不用加锁 */

    if (len == DT7_FRAME_LEN && dt7_parse(data, &new_data))
    {
        new_data.online = true;                             /* 收到合法帧：恢复在线 */
        new_data.frames++;
        uint32_t irq_key = os_irq_save();

        current_data = new_data;
        os_irq_restore(irq_key);
    }
    else
    {
        current_data.bad_frames++;
    }
}
EVENT_TASK_DEFINE(dt7_task, OS_PRIORITY_HIGH, OS_STACK_MEDIUM, EVENT_LATEST, 36, 1, DT7_TIMEOUT_MS, dt7_run, NULL);

void dt7_get(dt7_data_t *out)
{
    uint32_t irq_key = os_irq_save();

    *out = current_data;
    os_irq_restore(irq_key);
}

static int32_t dt7_init(void)
{
    memset(&current_data, 0, sizeof current_data);
    bsp_uart_bind_event(UART_DBUS, &dt7_task);
    return ERR_OK;
}
INIT_CDD_STAGE1(dt7_init);
