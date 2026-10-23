#include <string.h>
#include "cdd/supercap/supercap.h"
#include "supercap_config.h"
#include "bsp/can/bsp_can.h"
#include "bsp/tick/bsp_tick.h"
#include "component/common/err.h"
#include "component/common/initcall.h"
#include "osal/osal.h"
#include "osal/event.h"

#define SUPERCAP_RX_ID          0x05Au
#define SUPERCAP_TX_ID          0x06Au
#define SUPERCAP_POWER_MIN_W    35u
#define SUPERCAP_POWER_MAX_W    250u
#define SUPERCAP_CLEAR_AFTER    5u
#define SUPERCAP_RESTART_AFTER  10u

typedef struct
{
    bool             got_frame;
    uint32_t         rx_ms;
    supercap_error_e error;
    float            chassis_power_w;
    uint16_t         chassis_power_limit_w;
    uint8_t          energy_raw;
    bool             dcdc_on;
    uint16_t         referee_power_w;
    uint16_t         referee_buffer_j;
    uint8_t          clear_count;
    uint8_t          restart_count;
} supercap_t;

static supercap_t supercap;

static uint16_t clamp_power(uint16_t power_w)
{
    if (power_w < SUPERCAP_POWER_MIN_W)
    {
        return SUPERCAP_POWER_MIN_W;
    }
    if (power_w > SUPERCAP_POWER_MAX_W)
    {
        return SUPERCAP_POWER_MAX_W;
    }
    return power_w;
}

static void decode(const uint8_t *data)
{
    float power_w;

    memcpy(&power_w, &data[1], sizeof power_w);
    supercap.error = (supercap_error_e)(data[0] & 0x03u);
    supercap.chassis_power_w = power_w;
    supercap.chassis_power_limit_w = (uint16_t)data[5] | ((uint16_t)data[6] << 8);
    supercap.energy_raw = data[7];
    supercap.got_frame = true;
    supercap.rx_ms = bsp_tick_ms();
}

static void supercap_on_frame(void *argument, const uint8_t *message, uint16_t len)
{
    can_event_t event;

    (void)argument;
    if (message == NULL || len != sizeof event)
    {
        return;
    }
    memcpy(&event, message, sizeof event);
    if (event.len < 8u || event.id != SUPERCAP_RX_ID || event.bus != (uint8_t)SUPERCAP_BUS)
    {
        return;
    }
    uint32_t irq_key = os_irq_save();

    decode(event.data);
    os_irq_restore(irq_key);
}
EVENT_TASK_DEFINE(supercap_task, OS_PRIORITY_HIGHEST, OS_STACK_MEDIUM, EVENT_QUEUE, sizeof(can_event_t), OS_QUEUE_LEN_8, OS_TIMEOUT_NONE, supercap_on_frame, NULL);

static int32_t supercap_init(void)
{
    supercap.referee_power_w = SUPERCAP_POWER_W;
    supercap.referee_buffer_j = SUPERCAP_BUFFER_J;
    supercap.dcdc_on = true;
    return bsp_can_subscribe_event(SUPERCAP_BUS, SUPERCAP_RX_ID, SUPERCAP_RX_ID, &supercap_task);
}
INIT_CDD_STAGE1(supercap_init);

void supercap_enable(bool on)
{
    supercap.dcdc_on = on;
}

void supercap_set_referee(uint16_t power_limit_w, uint16_t buffer_j)
{
    supercap.referee_power_w = clamp_power(power_limit_w);
    supercap.referee_buffer_j = buffer_j;
}

void supercap_get(supercap_status_t *out)
{
    uint32_t irq_key = os_irq_save();
    bool fresh = supercap.got_frame && (bsp_tick_ms() - supercap.rx_ms) < SUPERCAP_TIMEOUT_MS;

    out->online = fresh;
    out->error = supercap.error;
    out->chassis_power_w = supercap.chassis_power_w;
    out->chassis_power_limit_w = supercap.chassis_power_limit_w;
    out->energy_raw = supercap.energy_raw;
    out->energy_j = (float)supercap.energy_raw * (SUPERCAP_ENERGY_FULL_J / 255.0f);
    os_irq_restore(irq_key);
}

static void recover(uint8_t *clear_error, uint8_t *restart)
{
    *clear_error = 0u;
    *restart = 0u;
    if (supercap.error != SUPERCAP_ERROR_CMD)
    {
        supercap.clear_count = 0u;
        supercap.restart_count = 0u;
        return;
    }
    supercap.clear_count++;
    if (supercap.clear_count <= SUPERCAP_CLEAR_AFTER)
    {
        return;
    }
    *clear_error = 1u;
    supercap.clear_count = 0u;
    supercap.restart_count++;
    if (supercap.restart_count >= SUPERCAP_RESTART_AFTER)
    {
        *restart = 1u;
        supercap.restart_count = 0u;
    }
}

int32_t supercap_send(void)
{
    can_frame_t frame;
    uint8_t clear_error;
    uint8_t restart;

    recover(&clear_error, &restart);
    memset(frame.data, 0, sizeof frame.data);
    frame.id = SUPERCAP_TX_ID;
    frame.len = 8;
    frame.data[0] = (uint8_t)((supercap.dcdc_on ? 0x01u : 0u) | (restart << 1) | (clear_error << 2));
    frame.data[1] = (uint8_t)supercap.referee_power_w;
    frame.data[2] = (uint8_t)(supercap.referee_power_w >> 8);
    frame.data[3] = (uint8_t)supercap.referee_buffer_j;
    frame.data[4] = (uint8_t)(supercap.referee_buffer_j >> 8);
    return bsp_can_send(SUPERCAP_BUS, &frame);
}
