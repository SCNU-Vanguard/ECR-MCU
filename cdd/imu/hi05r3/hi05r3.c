#include <string.h>
#include "cdd/imu/hi05r3/hi05r3.h"
#include "bsp/include/bsp_ids.h"
#include "ins_config.h"
#include "bsp/can/bsp_can.h"
#include "bsp/tick/bsp_tick.h"
#include "component/common/err.h"
#include "component/common/initcall.h"
#include "osal/osal.h"

#ifndef HI05R3_BUS
#error "ins_config.h must define HI05R3_BUS"
#endif
_Static_assert(HI05R3_BUS >= CAN_BUS1 && HI05R3_BUS < CAN_NUM, "HI05R3_BUS must be CAN_BUS1, CAN_BUS2, or CAN_BUS3");
#if HI05R3_NODE_ID < 1u || HI05R3_NODE_ID > 127u
#error "HI05R3_NODE_ID must be 1..127 (same range hi05r3_set accepts)"
#endif
_Static_assert(HI05R3_NODE_ID != HI05R3_HOST_ADDRESS, "HI05R3 node address must differ from J1939 host address");
#if HI05R3_RATE_HZ < 1u || HI05R3_RATE_HZ > 1000u
#error "HI05R3_RATE_HZ must be 1..1000"
#endif

/* 比例与 HiPNUC J1939 手册一致：加速度 1/2048 G（G 取 9.8），角速度 2000/32768 deg/s，角度 0.001 deg，四元数 0.0001。 */
#define ACC_LSB_M_S2            (9.8f / 2048.0f)
#define GYR_LSB_RAD_S           ((2000.0f / 32768.0f) * 0.017453292519943295f)
#define MDEG_TO_RAD             (0.001f * 0.017453292519943295f)
#define QUAT_LSB                0.0001f
#define RAW_CLIP                32760
#define J1939_PRIORITY          3u
#define PGN_ACC                 0xFF34u
#define PGN_GYR                 0xFF37u
#define PGN_ROLL_PITCH          0xFF3Du
#define PGN_YAW                 0xFF41u
#define PGN_TEMP                0xFF43u
#define PGN_QUAT                0xFF46u
#define PGN_INCLI               0xFF4Au
/* 设备欧拉角和倾角只给调试查看（getter），不随姿态发布；样本超过这个时间就当作没有。 */
#define DEBUG_VIEW_MAX_AGE_US        1000000u
/* 配置帧：PGN 0xEF00（PDU1，目的地址在 PS 字段）。主机地址 0x55，模块地址 HI05R3_NODE_ID。 */
#define CFG_RESPONSE_ID         ((J1939_PRIORITY << 26) | (0xEF00u << 8) | (HI05R3_HOST_ADDRESS << 8) | HI05R3_NODE_ID)
#define CFG_REQUEST_ID          ((J1939_PRIORITY << 26) | (0xEF00u << 8) | (HI05R3_NODE_ID << 8) | HI05R3_HOST_ADDRESS)
#define CFG_CMD_READ            0x03u
#define CFG_CMD_WRITE           0x06u
#define CFG_TIMEOUT_MS          100u
#define CFG_FRAME_GAP_MS        10u
/* 同一输出周期的几条报文紧挨着到，但谁先谁后不固定。放宽到 1.5 个周期，避免角速度先到时配不上刚到的姿态。 */
#define GROUP_WINDOW_US         ((1000000u / HI05R3_RATE_HZ) * 3u / 2u)

typedef struct
{
    float acc[3];
    float gyro[3];
    float quat[4];
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float inclinometer_x_rad;
    float temperature_c;
    uint32_t acc_us;
    uint32_t gyro_us;
    uint32_t roll_us;
    uint32_t yaw_us;
    uint32_t quat_us;
    uint32_t inclinometer_us;
    bool acc_valid;
    bool roll_valid;
    bool yaw_valid;
    bool quat_valid;
    bool inclinometer_valid;
    bool temperature_valid;
    bool gyro_clipped;
    bool accel_clipped;
#if HI05R3_USE_DEVICE_ATTITUDE
    uint32_t published_acc_count;
    uint32_t published_gyro_count;
    uint32_t published_quat_count;
#endif
    hi05r3_stats_t stats;
    hi05r3_frame_callback_t callback;
    void *context;
} hi05r3_state_t;

static hi05r3_state_t state;
static os_mutex_t config_mutex;
static volatile bool config_pending;
static volatile bool config_ready;
static volatile uint16_t config_address;
static volatile uint16_t config_value;

static void publish(uint32_t now_us)
{
    hi05r3_frame_t frame;
    float quat_norm2 = 0.0f;
    bool quaternion_fresh = state.quat_valid && (uint32_t)(now_us - state.quat_us) <= GROUP_WINDOW_US;
    bool attitude_fresh;

#if HI05R3_USE_DEVICE_ATTITUDE
    bool accel_fresh = state.acc_valid && (uint32_t)(now_us - state.acc_us) <= GROUP_WINDOW_US;
    bool gyro_fresh = (uint32_t)(now_us - state.gyro_us) <= GROUP_WINDOW_US;

    if (state.stats.acc_count == state.published_acc_count ||
        state.stats.gyro_count == state.published_gyro_count ||
        state.stats.quat_count == state.published_quat_count ||
        !accel_fresh || !gyro_fresh || !quaternion_fresh)
    {
        return;
    }
#endif
    if (!state.acc_valid || state.callback == NULL)
    {
        return;
    }
    memcpy(frame.gyro_rad_s, state.gyro, sizeof frame.gyro_rad_s);
    memcpy(frame.accel_m_s2, state.acc, sizeof frame.accel_m_s2);
    frame.temperature_c = state.temperature_c;
    frame.timestamp_us = state.gyro_us;
    frame.communication_time_us = 0u;
    frame.accel_fresh = (uint32_t)(now_us - state.acc_us) <= GROUP_WINDOW_US;
    frame.temperature_valid = state.temperature_valid;
    frame.gyro_clipped = state.gyro_clipped;
    frame.accel_clipped = state.accel_clipped;
    quat_norm2 = state.quat[0] * state.quat[0] + state.quat[1] * state.quat[1] +
                 state.quat[2] * state.quat[2] + state.quat[3] * state.quat[3];
    attitude_fresh = frame.accel_fresh && quaternion_fresh && quat_norm2 > 0.25f && quat_norm2 < 4.0f;
#if HI05R3_USE_DEVICE_ATTITUDE
    if (!attitude_fresh)
    {
        return;
    }
    state.published_acc_count = state.stats.acc_count;
    state.published_gyro_count = state.stats.gyro_count;
    state.published_quat_count = state.stats.quat_count;
#endif
    state.stats.publish_count++;
    frame.sequence = state.stats.gyro_count;
    frame.attitude_valid = attitude_fresh;
    memset(frame.quat, 0, sizeof frame.quat);
    frame.quat[0] = 1.0f;
    if (attitude_fresh)
    {
        const uint32_t accel_age_us = (uint32_t)(now_us - state.acc_us);
        const uint32_t gyro_age_us = (uint32_t)(now_us - state.gyro_us);
        const uint32_t quat_age_us = (uint32_t)(now_us - state.quat_us);
        frame.communication_time_us = accel_age_us > quat_age_us ? accel_age_us : quat_age_us;
        if (gyro_age_us > frame.communication_time_us)
        {
            frame.communication_time_us = gyro_age_us;
        }
        memcpy(frame.quat, state.quat, sizeof frame.quat);
    }
    state.callback(&frame, state.context);
}

static void on_can_frame(can_id_e bus, const can_frame_t *frame, void *context)
{
    uint32_t pgn;
    uint32_t now_us;

    (void)bus;
    (void)context;
    if (frame == NULL || frame->id > 0x1FFFFFFFu)
    {
        return;
    }
    if (frame->id == CFG_RESPONSE_ID)
    {
        if (frame->len >= 8u && frame->data[2] == CFG_CMD_READ && config_pending &&
            ((uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8)) == config_address)
        {
            config_value = (uint16_t)frame->data[4] | ((uint16_t)frame->data[5] << 8);
            config_ready = true;
        }
        return;
    }
    if ((frame->id & 0xFFu) != (uint32_t)(HI05R3_NODE_ID & 0xFFu))
    {
        return;
    }
    state.stats.rx_frames++;
    pgn = (frame->id >> 8) & 0x3FFFFu;
    if (((pgn >> 8) & 0xFFu) < 0xF0u)
    {
        pgn &= 0x3FF00u;
    }
    now_us = (uint32_t)bsp_tick_us();
    switch (pgn)
    {
    case PGN_ACC:
        if (frame->len < 6u)
        {
            state.stats.ignored_count++;
            return;
        }
        state.accel_clipped = false;
        for (uint32_t axis = 0u; axis < 3u; axis++)
        {
            const uint8_t *data = frame->data + 2u * axis;
            int16_t raw = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));

            state.acc[axis] = (float)raw * ACC_LSB_M_S2;
            state.accel_clipped |= raw >= RAW_CLIP || raw <= -RAW_CLIP;
        }
        state.acc_us = now_us;
        state.acc_valid = true;
        state.stats.acc_count++;
#if HI05R3_USE_DEVICE_ATTITUDE
        publish(now_us);
#endif
        break;
    case PGN_GYR:
        if (frame->len < 6u)
        {
            state.stats.ignored_count++;
            return;
        }
        state.gyro_clipped = false;
        for (uint32_t axis = 0u; axis < 3u; axis++)
        {
            const uint8_t *data = frame->data + 2u * axis;
            int16_t raw = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));

            state.gyro[axis] = (float)raw * GYR_LSB_RAD_S;
            state.gyro_clipped |= raw >= RAW_CLIP || raw <= -RAW_CLIP;
        }
        state.gyro_us = now_us;
        state.stats.gyro_count++;
        publish(now_us);
        break;
    case PGN_ROLL_PITCH:
        if (frame->len < 8u)
        {
            state.stats.ignored_count++;
            return;
        }
        for (uint32_t index = 0u; index < 2u; index++)
        {
            const uint8_t *data = frame->data + 4u * index;
            const int32_t raw = (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                                          ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
            float *angle = index == 0u ? &state.roll_rad : &state.pitch_rad;

            *angle = (float)raw * MDEG_TO_RAD;
        }
        state.roll_us = now_us;
        state.roll_valid = true;
        state.stats.attitude_count++;
        break;
    case PGN_YAW:
        if (frame->len < 4u)
        {
            state.stats.ignored_count++;
            return;
        }
        {
            /* 8 字节时第二段是逆时针欧拉航向；只有 4 字节时是顺时针 0..360° 航向。 */
            const bool has_euler_yaw = frame->len >= 8u;
            const uint8_t *data = frame->data + (has_euler_yaw ? 4u : 0u);
            const int32_t raw = (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                                          ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));

            state.yaw_rad = (has_euler_yaw ? (float)raw : -(float)raw) * MDEG_TO_RAD;
        }
        state.yaw_us = now_us;
        state.yaw_valid = true;
        break;
    case PGN_TEMP:
        if (frame->len < 2u)
        {
            state.stats.ignored_count++;
            return;
        }
        state.temperature_c = (float)(int16_t)((uint16_t)frame->data[0] |
                                               ((uint16_t)frame->data[1] << 8)) * 0.01f;
        state.temperature_valid = true;
        state.stats.temperature_count++;
        break;
    case PGN_QUAT:
        if (frame->len < 8u)
        {
            state.stats.ignored_count++;
            return;
        }
        for (uint32_t index = 0u; index < 4u; index++)
        {
            const uint8_t *data = frame->data + 2u * index;
            int16_t raw = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));

            state.quat[index] = (float)raw * QUAT_LSB;
        }
        state.quat_us = now_us;
        state.quat_valid = true;
        state.stats.quat_count++;
#if HI05R3_USE_DEVICE_ATTITUDE
        publish(now_us);
#endif
        break;
    case PGN_INCLI:
        if (frame->len < 4u)
        {
            state.stats.ignored_count++;
            return;
        }
        state.inclinometer_x_rad = (float)(int32_t)((uint32_t)frame->data[0] | ((uint32_t)frame->data[1] << 8) |
                                                   ((uint32_t)frame->data[2] << 16) |
                                                   ((uint32_t)frame->data[3] << 24)) * MDEG_TO_RAD;
        state.inclinometer_us = now_us;
        state.inclinometer_valid = true;
        state.stats.inclinometer_count++;
        break;
    default:
        state.stats.ignored_count++;
        break;
    }
}

void hi05r3_set_frame_callback(hi05r3_frame_callback_t callback, void *context)
{
    uint32_t irq_key = os_irq_save();

    state.context = context;
    state.callback = callback;
    os_irq_restore(irq_key);
}

static int32_t hi05r3_init(void)
{
    int32_t ret = os_mutex_init(&config_mutex);

    if (ret != ERR_OK)
    {
        return ret;
    }
    ret = bsp_can_subscribe(HI05R3_BUS, CFG_RESPONSE_ID, CFG_RESPONSE_ID, on_can_frame, NULL);
    if (ret != ERR_OK)
    {
        return ret;
    }
    const uint32_t first_id = (J1939_PRIORITY << 26) | (PGN_ACC << 8) | HI05R3_NODE_ID;
    const uint32_t last_id = (J1939_PRIORITY << 26) | (PGN_INCLI << 8) | HI05R3_NODE_ID;

    return bsp_can_subscribe(HI05R3_BUS, first_id, last_id, on_can_frame, NULL);
}
INIT_CDD_STAGE1(hi05r3_init);

static int32_t hi05r3_config_send(uint16_t address, uint8_t command, uint16_t value)
{
    const uint8_t data[8] = {
        (uint8_t)(address & 0xFFu), (uint8_t)(address >> 8), command, 0u,
        (uint8_t)(value & 0xFFu), (uint8_t)(value >> 8), 0u, 0u,
    };
    int32_t ret = bsp_can_send_extended(HI05R3_BUS, CFG_REQUEST_ID, data, 8u);

    /* 设备手册要求相邻配置帧至少间隔 10 ms。 */
    if (ret == ERR_OK)
    {
        os_delay_ms(CFG_FRAME_GAP_MS);
    }
    return ret;
}

int32_t hi05r3_set_register(uint16_t address, uint16_t value)
{
    if (address == HI05R3_CTRL_ADDR && value != HI05R3_CTRL_SAVE && value != HI05R3_CTRL_REBOOT)
    {
        return ERR_INVAL;
    }
    int32_t ret = os_mutex_lock(&config_mutex, OS_WAIT_FOREVER);

    if (ret != ERR_OK)
    {
        return ret;
    }
    ret = hi05r3_config_send(address, CFG_CMD_WRITE, value);
    os_mutex_unlock(&config_mutex);
    return ret;
}

int32_t hi05r3_set_read_register(uint16_t address, uint16_t *value)
{
    if (value == NULL || address == HI05R3_CTRL_ADDR)
    {
        return ERR_INVAL;
    }
    int32_t ret = os_mutex_lock(&config_mutex, OS_WAIT_FOREVER);

    if (ret != ERR_OK)
    {
        return ret;
    }
    uint32_t irq_key = os_irq_save();

    config_ready = false;
    config_address = address;
    config_pending = true;
    os_irq_restore(irq_key);

    ret = hi05r3_config_send(address, CFG_CMD_READ, 0u);
    if (ret == ERR_OK)
    {
        ret = ERR_TIMEOUT;
        for (uint32_t elapsed = 0u; elapsed < CFG_TIMEOUT_MS && ret != ERR_OK; elapsed++)
        {
            os_delay_ms(1u);
            irq_key = os_irq_save();
            if (config_ready)
            {
                *value = config_value;
                ret = ERR_OK;
            }
            os_irq_restore(irq_key);
        }
    }
    irq_key = os_irq_save();
    config_pending = false;
    config_ready = false;
    os_irq_restore(irq_key);
    os_mutex_unlock(&config_mutex);
    return ret;
}

bool hi05r3_get_euler(float *roll_rad, float *pitch_rad, float *yaw_rad)
{
    const uint32_t now_us = (uint32_t)bsp_tick_us();
    uint32_t irq_key = os_irq_save();
    const bool valid = state.roll_valid && state.yaw_valid &&
                       (uint32_t)(now_us - state.roll_us) <= DEBUG_VIEW_MAX_AGE_US &&
                       (uint32_t)(now_us - state.yaw_us) <= DEBUG_VIEW_MAX_AGE_US;

    if (valid)
    {
        if (roll_rad != NULL)
        {
            *roll_rad = state.roll_rad;
        }
        if (pitch_rad != NULL)
        {
            *pitch_rad = state.pitch_rad;
        }
        if (yaw_rad != NULL)
        {
            *yaw_rad = state.yaw_rad;
        }
    }
    os_irq_restore(irq_key);
    return valid;
}

bool hi05r3_get_inclinometer(float *x_rad)
{
    const uint32_t now_us = (uint32_t)bsp_tick_us();
    uint32_t irq_key = os_irq_save();
    const bool valid = state.inclinometer_valid && (uint32_t)(now_us - state.inclinometer_us) <= DEBUG_VIEW_MAX_AGE_US;

    if (valid && x_rad != NULL)
    {
        *x_rad = state.inclinometer_x_rad;
    }
    os_irq_restore(irq_key);
    return valid;
}

void hi05r3_get_stats(hi05r3_stats_t *stats)
{
    uint32_t irq_key = os_irq_save();

    *stats = state.stats;
    os_irq_restore(irq_key);
}
