#include <math.h>
#include "cdd/imu/hi05r3/hi05r3.h"
#include "bsp/include/bsp_ids.h"
#include "ins_config.h"
#include "bsp/can/bsp_can.h"
#include "bsp/tick/bsp_tick.h"
#include "osal/osal.h"
#include "component/common/err.h"

#ifndef HI05R3_BUS
#error "ins_config.h must define HI05R3_BUS"
#endif
_Static_assert(HI05R3_BUS >= CAN_BUS1 && HI05R3_BUS < CAN_NUM, "HI05R3_BUS must be CAN_BUS1, CAN_BUS2, or CAN_BUS3");
#if HI05R3_NODE_ID > 255u
#error "HI05R3_NODE_ID must fit in the J1939 source address"
#endif
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
/* 同一输出周期的几条报文紧挨着到，但谁先谁后不固定。放宽到 1.5 个周期，避免角速度先到时配不上刚到的姿态。 */
#define GROUP_WINDOW_US         ((1000000u / HI05R3_RATE_HZ) * 3u / 2u)

typedef struct
{
    float acc[3];
    float gyro[3];
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float quat[4];
    float temperature_c;
    uint32_t acc_us;
    uint32_t roll_us;
    uint32_t yaw_us;
    uint32_t quat_us;
    bool acc_valid;
    bool roll_valid;
    bool yaw_valid;
    bool quat_valid;
    bool temperature_valid;
    bool gyro_clipped;
    bool accel_clipped;
    hi05r3_stats_t stats;
    hi05r3_frame_callback_t callback;
    void *context;
} hi05r3_state_t;

static hi05r3_state_t state;

static int16_t read_i16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int32_t read_i32(const uint8_t *data)
{
    return (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
}

static uint32_t j1939_id(uint32_t pgn)
{
    return (J1939_PRIORITY << 26) | ((pgn & 0x3FFFFu) << 8) | (uint32_t)(HI05R3_NODE_ID & 0xFFu);
}

static uint32_t pgn_of(uint32_t id)
{
    uint32_t pgn = (id >> 8) & 0x3FFFFu;

    if (((pgn >> 8) & 0xFFu) < 0xF0u)
    {
        pgn &= 0x3FF00u;
    }
    return pgn;
}

static bool fresh(uint32_t now_us, uint32_t stamp_us, bool valid)
{
    return valid && (uint32_t)(now_us - stamp_us) <= GROUP_WINDOW_US;
}

static bool clipped3(int16_t x, int16_t y, int16_t z)
{
    return x >= RAW_CLIP || x <= -RAW_CLIP || y >= RAW_CLIP || y <= -RAW_CLIP || z >= RAW_CLIP || z <= -RAW_CLIP;
}

static float wrap_pi(float angle)
{
    const float pi = 3.14159265f;
    const float two_pi = 6.2831853f;

    if (angle > pi || angle < -pi)
    {
        angle = fmodf(angle, two_pi);
        if (angle > pi)
        {
            angle -= two_pi;
        }
        else if (angle < -pi)
        {
            angle += two_pi;
        }
    }
    return angle;
}

/* ZYX（yaw-pitch-roll），和 component/ahrs 从四元数反解欧拉角的约定一致。 */
static void euler_to_quat(float roll, float pitch, float yaw, float quat[4])
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    quat[0] = cr * cp * cy + sr * sp * sy;
    quat[1] = sr * cp * cy - cr * sp * sy;
    quat[2] = cr * sp * cy + sr * cp * sy;
    quat[3] = cr * cp * sy - sr * sp * cy;
}

static void publish(uint32_t now_us)
{
    hi05r3_frame_t frame;
    bool attitude_fresh = fresh(now_us, state.roll_us, state.roll_valid) && fresh(now_us, state.yaw_us, state.yaw_valid);

    if (!state.acc_valid || state.callback == NULL)
    {
        return;
    }
    frame.gyro_rad_s[0] = state.gyro[0];
    frame.gyro_rad_s[1] = state.gyro[1];
    frame.gyro_rad_s[2] = state.gyro[2];
    frame.accel_m_s2[0] = state.acc[0];
    frame.accel_m_s2[1] = state.acc[1];
    frame.accel_m_s2[2] = state.acc[2];
    frame.temperature_c = state.temperature_c;
    frame.timestamp_us = now_us;
    state.stats.publish_count++;
    frame.sequence = state.stats.publish_count;
    frame.accel_fresh = fresh(now_us, state.acc_us, true);
    frame.temperature_valid = state.temperature_valid;
    frame.gyro_clipped = state.gyro_clipped;
    frame.accel_clipped = state.accel_clipped;
    frame.attitude_valid = attitude_fresh;
    frame.quat_from_device = false;
    frame.roll_rad = 0.0f;
    frame.pitch_rad = 0.0f;
    frame.yaw_rad = 0.0f;
    frame.quat[0] = 1.0f;
    frame.quat[1] = 0.0f;
    frame.quat[2] = 0.0f;
    frame.quat[3] = 0.0f;
    if (attitude_fresh)
    {
        frame.roll_rad = state.roll_rad;
        frame.pitch_rad = state.pitch_rad;
        frame.yaw_rad = wrap_pi(state.yaw_rad);
        if (fresh(now_us, state.quat_us, state.quat_valid))
        {
            float norm2 = state.quat[0] * state.quat[0] + state.quat[1] * state.quat[1] +
                          state.quat[2] * state.quat[2] + state.quat[3] * state.quat[3];

            if (norm2 > 0.25f && norm2 < 4.0f)
            {
                frame.quat[0] = state.quat[0];
                frame.quat[1] = state.quat[1];
                frame.quat[2] = state.quat[2];
                frame.quat[3] = state.quat[3];
                frame.quat_from_device = true;
            }
            else
            {
                euler_to_quat(frame.roll_rad, frame.pitch_rad, frame.yaw_rad, frame.quat);
            }
        }
        else
        {
            euler_to_quat(frame.roll_rad, frame.pitch_rad, frame.yaw_rad, frame.quat);
        }
    }
    state.callback(&frame, state.context);
}

static void on_can_frame(can_id_e bus, const can_frame_t *frame, void *context)
{
    uint32_t pgn;
    uint32_t now_us;

    (void)bus;
    (void)context;
    if (frame == NULL || frame->id > 0x1FFFFFFFu || (frame->id & 0xFFu) != (uint32_t)(HI05R3_NODE_ID & 0xFFu))
    {
        return;
    }
    state.stats.rx_frames++;
    pgn = pgn_of(frame->id);
    now_us = (uint32_t)bsp_tick_us();
    switch (pgn)
    {
    case PGN_ACC:
        if (frame->len < 6u)
        {
            state.stats.ignored_count++;
            return;
        }
        state.accel_clipped = clipped3(read_i16(frame->data), read_i16(frame->data + 2), read_i16(frame->data + 4));
        state.acc[0] = (float)read_i16(frame->data) * ACC_LSB_M_S2;
        state.acc[1] = (float)read_i16(frame->data + 2) * ACC_LSB_M_S2;
        state.acc[2] = (float)read_i16(frame->data + 4) * ACC_LSB_M_S2;
        state.acc_us = now_us;
        state.acc_valid = true;
        state.stats.acc_count++;
        break;
    case PGN_GYR:
        if (frame->len < 6u)
        {
            state.stats.ignored_count++;
            return;
        }
        state.gyro_clipped = clipped3(read_i16(frame->data), read_i16(frame->data + 2), read_i16(frame->data + 4));
        state.gyro[0] = (float)read_i16(frame->data) * GYR_LSB_RAD_S;
        state.gyro[1] = (float)read_i16(frame->data + 2) * GYR_LSB_RAD_S;
        state.gyro[2] = (float)read_i16(frame->data + 4) * GYR_LSB_RAD_S;
        state.stats.gyro_count++;
        publish(now_us);
        break;
    case PGN_ROLL_PITCH:
        if (frame->len < 8u)
        {
            state.stats.ignored_count++;
            return;
        }
        state.roll_rad = (float)read_i32(frame->data) * MDEG_TO_RAD;
        state.pitch_rad = (float)read_i32(frame->data + 4) * MDEG_TO_RAD;
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
        if (frame->len >= 8u)
        {
            state.yaw_rad = (float)read_i32(frame->data + 4) * MDEG_TO_RAD; /* 第二段是设备欧拉航向，逆时针 */
        }
        else
        {
            state.yaw_rad = -(float)read_i32(frame->data) * MDEG_TO_RAD;    /* 只有顺时针 0..360° 航向 */
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
        state.temperature_c = (float)read_i16(frame->data) * 0.01f;
        state.temperature_valid = true;
        state.stats.temperature_count++;
        break;
    case PGN_QUAT:
        if (frame->len < 8u)
        {
            state.stats.ignored_count++;
            return;
        }
        state.quat[0] = (float)read_i16(frame->data) * QUAT_LSB;
        state.quat[1] = (float)read_i16(frame->data + 2) * QUAT_LSB;
        state.quat[2] = (float)read_i16(frame->data + 4) * QUAT_LSB;
        state.quat[3] = (float)read_i16(frame->data + 6) * QUAT_LSB;
        state.quat_us = now_us;
        state.quat_valid = true;
        state.stats.quat_count++;
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

int32_t hi05r3_start(void)
{
    return bsp_can_subscribe(HI05R3_BUS, j1939_id(PGN_ACC), j1939_id(PGN_QUAT), on_can_frame, NULL);
}

void hi05r3_get_stats(hi05r3_stats_t *stats)
{
    uint32_t irq_key = os_irq_save();

    *stats = state.stats;
    os_irq_restore(irq_key);
}
