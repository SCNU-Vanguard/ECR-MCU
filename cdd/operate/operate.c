#include <string.h>
#include "cdd/operate/operate.h"
#include "operate_config.h"
#if OPERATE_MODE == OPERATE_MODE_IMGTRANS_KEY
#include <math.h>
#endif
#include "bsp/tick/bsp_tick.h"
#include "osal/period.h"
#include "osal/osal.h"

#if OPERATE_MODE == OPERATE_MODE_DT7
#include "cdd/operate/remote/dt7/dt7.h"
#define DT7_SWITCH_UP   1u
#define DT7_SWITCH_MID  3u
#elif OPERATE_MODE == OPERATE_MODE_IMGTRANS_RC || OPERATE_MODE == OPERATE_MODE_IMGTRANS_KEY
#include "cdd/operate/imgtrans/imgtrans.h"
#else
#error "OPERATE_MODE is not DT7, imgtrans remote, or imgtrans keyboard"
#endif

#ifndef OPERATE_REG_TABLE
#error "operate_config.h must define OPERATE_REG_TABLE"
#endif

#define STICK_FULL  660.0f

typedef struct
{
    uint8_t  id;
    uint8_t  attr;
    uint8_t  rc;
    uint32_t bind;
} operate_reg_t;

#define OPERATE_REG(id, bind, attr, rc) \
    { (uint8_t)(id), (uint8_t)(attr), (uint8_t)(rc), (uint32_t)(bind) },
static const operate_reg_t operate_regs[] = {
    OPERATE_REG_TABLE
};
#undef OPERATE_REG

typedef struct
{
    bool     online;
    bool     enabled;
    float    forward_m_s;
    float    right_m_s;
    float    spin_rad_s;
    bool     use_nav;
    bool     use_aim;
    float    yaw_delta_rad;
    float    pitch_delta_rad;
    uint32_t seq;
    bool     friction;
    bool     shoot;
    bool     custom;
} operate_snap_t;

static operate_snap_t published;
static uint32_t       last_frames;
static uint64_t       frame_us;
static bool           frame_stamp;
static bool           reg_on[OPERATE_IN_COUNT];
#if OPERATE_MODE != OPERATE_MODE_IMGTRANS_RC
static bool           reg_raw_prev[OPERATE_IN_COUNT];
#endif

typedef struct
{
    uint16_t size;
    uint16_t offset;
} operate_field_t;

#define OPERATE_FIELD(member) \
    { (uint16_t)sizeof(((operate_snap_t *)0)->member), (uint16_t)offsetof(operate_snap_t, member) }

static const operate_field_t operate_field[OPERATE_ID_COUNT] = {
    [OPERATE_ID_ONLINE]      = OPERATE_FIELD(online),
    [OPERATE_ID_ENABLE]      = OPERATE_FIELD(enabled),
    [OPERATE_ID_FORWARD]     = OPERATE_FIELD(forward_m_s),
    [OPERATE_ID_RIGHT]       = OPERATE_FIELD(right_m_s),
    [OPERATE_ID_SPIN]        = OPERATE_FIELD(spin_rad_s),
    [OPERATE_ID_NAV]         = OPERATE_FIELD(use_nav),
    [OPERATE_ID_AIM]         = OPERATE_FIELD(use_aim),
    [OPERATE_ID_YAW_DELTA]   = OPERATE_FIELD(yaw_delta_rad),
    [OPERATE_ID_PITCH_DELTA] = OPERATE_FIELD(pitch_delta_rad),
    [OPERATE_ID_SEQ]         = OPERATE_FIELD(seq),
    [OPERATE_ID_FRICTION]    = OPERATE_FIELD(friction),
    [OPERATE_ID_SHOOT]       = OPERATE_FIELD(shoot),
    [OPERATE_ID_CUSTOM]      = OPERATE_FIELD(custom),
};
#undef OPERATE_FIELD

#if OPERATE_MODE != OPERATE_MODE_IMGTRANS_KEY
static float stick(int16_t raw, float scale)
{
    if (raw > -OPERATE_STICK_DEADZONE && raw < OPERATE_STICK_DEADZONE)
    {
        return 0.0f;
    }
    return (float)raw / STICK_FULL * scale;
}

static float wheel_spin(int16_t raw)
{
    if (raw > OPERATE_WHEEL_DEADZONE)
    {
        return OPERATE_SPIN_RAD_S;
    }
    if (raw < -OPERATE_WHEEL_DEADZONE)
    {
        return -OPERATE_SPIN_RAD_S;
    }
    return 0.0f;
}
#endif

static void publish(operate_snap_t *out)
{
    uint32_t irq_key = os_irq_save();

    published = *out;
    os_irq_restore(irq_key);
}

/* 上一帧时间戳到现在的秒数。还没有时间戳时返回 0，并记下当前时刻。 */
static float frame_interval_s(void)
{
    uint64_t now = bsp_tick_us();
    float interval = 0.0f;

    if (frame_stamp)
    {
        interval = (float)(now - frame_us) * 1.0e-6f;
    }
    frame_us = now;
    frame_stamp = true;
    return interval;
}

/* 新帧才推进 seq。增量是上一帧到这一帧摇杆转过的角，再加上这一帧鼠标。 */
static void note_frame(operate_snap_t *out, bool new_frame, float yaw_rate, float pitch_rate,
                       float mouse_yaw, float mouse_pitch)
{
    float interval;

    if (!new_frame)
    {
        out->yaw_delta_rad = published.yaw_delta_rad;
        out->pitch_delta_rad = published.pitch_delta_rad;
        return;
    }
    interval = frame_interval_s();
    out->yaw_delta_rad = yaw_rate * interval + mouse_yaw;
    out->pitch_delta_rad = pitch_rate * interval + mouse_pitch;
    out->seq = published.seq + 1u;
}

#if OPERATE_MODE != OPERATE_MODE_IMGTRANS_RC
static uint32_t pointer_bits(uint16_t key, uint8_t left, uint8_t right, uint8_t middle)
{
    uint32_t bits = key;

    if (left != 0u)
    {
        bits |= OPERATE_MOUSE_LEFT;
    }
    if (right != 0u)
    {
        bits |= OPERATE_MOUSE_RIGHT;
    }
    if (middle != 0u)
    {
        bits |= OPERATE_MOUSE_MIDDLE;
    }
    return bits;
}
#endif

#if OPERATE_MODE != OPERATE_MODE_IMGTRANS_RC
static bool bind_down(uint32_t bits, uint32_t bind)
{
    return bind != 0u && (bits & bind) == bind;
}
#endif

#if OPERATE_MODE == OPERATE_MODE_IMGTRANS_RC
static bool rc_down(uint8_t rc, const imgtrans_data_t *in)
{
    switch (rc)
    {
    case OPERATE_RC_PAUSE:
        return in->pause != 0u;
    case OPERATE_RC_CUSTOM_LEFT:
        return in->custom_left != 0u;
    case OPERATE_RC_CUSTOM_RIGHT:
        return in->custom_right != 0u;
    case OPERATE_RC_TRIGGER:
        return in->trigger != 0u;
    case OPERATE_RC_AIM:
        return (in->gear & 1u) != 0u;
    case OPERATE_RC_NAV:
        return (in->gear & 2u) != 0u;
    case OPERATE_RC_SPIN:
        return in->wheel > OPERATE_WHEEL_DEADZONE || in->wheel < -OPERATE_WHEEL_DEADZONE;
    default:
        return false;
    }
}
#endif

static void clear_regs(void)
{
    memset(reg_on, 0, sizeof reg_on);
#if OPERATE_MODE != OPERATE_MODE_IMGTRANS_RC
    memset(reg_raw_prev, 0, sizeof reg_raw_prev);
#endif
}

static bool reg_is_on(unsigned id)
{
    return id < OPERATE_IN_COUNT && reg_on[id];
}

#if OPERATE_MODE == OPERATE_MODE_IMGTRANS_RC
/* 新的一帧把遥控电平抄进登记项。 */
static void apply_regs(bool new_frame, const imgtrans_data_t *in)
{
    unsigned i;

    if (!new_frame)
    {
        return;
    }
    for (i = 0u; i < sizeof operate_regs / sizeof operate_regs[0]; i++)
    {
        const operate_reg_t *reg = &operate_regs[i];

        if (reg->id >= OPERATE_IN_COUNT)
        {
            continue;
        }
        reg_on[reg->id] = rc_down(reg->rc, in);
    }
}
#else
/* 新的一帧才采样键。按住跟着键，切换在上升沿翻转。 */
static void apply_regs(uint32_t bits, bool new_frame)
{
    unsigned i;

    if (!new_frame)
    {
        return;
    }
    for (i = 0u; i < sizeof operate_regs / sizeof operate_regs[0]; i++)
    {
        const operate_reg_t *reg = &operate_regs[i];
        unsigned id = reg->id;
        bool raw;
        bool was;

        if (id >= OPERATE_IN_COUNT)
        {
            continue;
        }
        raw = bind_down(bits, reg->bind);
        was = reg_raw_prev[id];
        reg_raw_prev[id] = raw;
        if (reg->attr == OPERATE_AS_TOGGLE)
        {
            if (raw && !was)
            {
                reg_on[id] = !reg_on[id];
            }
        }
        else
        {
            reg_on[id] = raw;
        }
    }
}
#endif

#if OPERATE_MODE == OPERATE_MODE_DT7
static void from_dt7(void)
{
    dt7_data_t remote;
    operate_snap_t out;
    bool new_frame;

    dt7_get(&remote);
    memset(&out, 0, sizeof out);
    out.seq = published.seq;
    out.online = remote.online;
    if (!remote.online)
    {
        clear_regs();
        frame_stamp = false;
        publish(&out);
        return;
    }
    /* 左开关：上或中为使能，上档同时开自瞄和导航。右开关：上或中为摩擦轮，上为拨弹。 */
    out.enabled = remote.switch_left == DT7_SWITCH_UP || remote.switch_left == DT7_SWITCH_MID;
    out.use_aim = remote.switch_left == DT7_SWITCH_UP;
    out.use_nav = remote.switch_left == DT7_SWITCH_UP;
    out.friction = remote.switch_right == DT7_SWITCH_UP || remote.switch_right == DT7_SWITCH_MID;
    out.shoot = remote.switch_right == DT7_SWITCH_UP;
    out.forward_m_s = stick(remote.channel[3], OPERATE_MOVE_MAX_M_S);
    out.right_m_s = stick(remote.channel[2], OPERATE_MOVE_MAX_M_S);
    out.spin_rad_s = wheel_spin(remote.wheel);
    new_frame = remote.frames != last_frames;
    apply_regs(pointer_bits(remote.key, remote.mouse_left, remote.mouse_right, 0u), new_frame);
    out.custom = reg_is_on(OPERATE_IN_CUSTOM);
    note_frame(&out, new_frame,
               stick(remote.channel[0], -OPERATE_YAW_RATE_RAD_S),
               stick(remote.channel[1], OPERATE_PITCH_RATE_RAD_S),
               0.0f, 0.0f);
    if (new_frame)
    {
        last_frames = remote.frames;
    }
    publish(&out);
}
#else
static void from_imgtrans(void)
{
    imgtrans_data_t in;
    operate_snap_t out;
    bool new_frame;

    imgtrans_get(&in);
    memset(&out, 0, sizeof out);
    out.seq = published.seq;
    out.online = in.online;
    if (!in.online)
    {
        clear_regs();
        frame_stamp = false;
        publish(&out);
        return;
    }
    new_frame = in.frames != last_frames;
#if OPERATE_MODE == OPERATE_MODE_IMGTRANS_RC
    apply_regs(new_frame, &in);
#else
    apply_regs(pointer_bits(in.key, in.mouse_left, in.mouse_right, in.mouse_middle), new_frame);
#endif
    note_frame(&out, new_frame,
#if OPERATE_MODE == OPERATE_MODE_IMGTRANS_RC
               stick(in.channel[0], -OPERATE_YAW_RATE_RAD_S),
               stick(in.channel[1], OPERATE_PITCH_RATE_RAD_S),
               0.0f, 0.0f);
#else
               0.0f, 0.0f,
               -(float)in.mouse_x * OPERATE_MOUSE_YAW_RAD_PER_COUNT,
               (float)in.mouse_y * OPERATE_MOUSE_PITCH_RAD_PER_COUNT);
#endif
    if (new_frame)
    {
        last_frames = in.frames;
    }

    out.enabled = reg_is_on(OPERATE_IN_ENABLE);
    out.use_aim = reg_is_on(OPERATE_IN_AIM);
    out.use_nav = reg_is_on(OPERATE_IN_NAV);
    out.friction = reg_is_on(OPERATE_IN_FRICTION);
    out.shoot = reg_is_on(OPERATE_IN_SHOOT);
    out.custom = reg_is_on(OPERATE_IN_CUSTOM);
#if OPERATE_MODE == OPERATE_MODE_IMGTRANS_RC
    out.forward_m_s = stick(in.channel[3], OPERATE_MOVE_MAX_M_S);
    out.right_m_s = stick(in.channel[2], OPERATE_MOVE_MAX_M_S);
    out.spin_rad_s = wheel_spin(in.wheel);
#else
    {
        float right = 0.0f;
        float forward = 0.0f;
        float magnitude;

        if (reg_is_on(OPERATE_IN_FORWARD))
        {
            forward += 1.0f;
        }
        if (reg_is_on(OPERATE_IN_BACK))
        {
            forward -= 1.0f;
        }
        if (reg_is_on(OPERATE_IN_RIGHT))
        {
            right += 1.0f;
        }
        if (reg_is_on(OPERATE_IN_LEFT))
        {
            right -= 1.0f;
        }
        magnitude = sqrtf(right * right + forward * forward);
        if (magnitude > 1.0f)
        {
            right /= magnitude;
            forward /= magnitude;
        }
        out.forward_m_s = forward * OPERATE_MOVE_MAX_M_S;
        out.right_m_s = right * OPERATE_MOVE_MAX_M_S;
        out.spin_rad_s = reg_is_on(OPERATE_IN_SPIN) ? OPERATE_SPIN_RAD_S : 0.0f;
    }
#endif
    publish(&out);
}
#endif

static void operate_run(void *argument)
{
    (void)argument;
#if OPERATE_MODE == OPERATE_MODE_DT7
    from_dt7();
#else
    from_imgtrans();
#endif
}
/* 输入大约 100 Hz。200 Hz 轮询每帧能看到一次，摇杆角用时间戳，不靠这个周期。 */
PERIOD_TASK_DEFINE(operate_task, OS_FREQ_200HZ, OS_PRIORITY_MEDIUM, OS_STACK_SMALL, operate_run, NULL);

int32_t operate_get(unsigned id, void *buf, size_t size)
{
    uint32_t irq_key;

    if (buf == NULL || id >= OPERATE_ID_COUNT || (size_t)operate_field[id].size != size)
    {
        return ERR_INVAL;
    }
    irq_key = os_irq_save();
    memcpy(buf, (const uint8_t *)&published + operate_field[id].offset, size);
    os_irq_restore(irq_key);
    return ERR_OK;
}

#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"

static bool shell_bool(unsigned id)
{
    bool value = false;

    (void)operate_get(id, &value, sizeof value);
    return value;
}

static float shell_float(unsigned id)
{
    float value = 0.0f;

    (void)operate_get(id, &value, sizeof value);
    return value;
}

static int shell_operate(int argc, char *argv[])
{
    uint32_t seq = 0u;

    (void)argc;
    (void)argv;
    (void)operate_get(OPERATE_ID_SEQ, &seq, sizeof seq);
    shell_out("operate: online=%d enabled=%d aim=%d nav=%d fric=%d shoot=%d custom=%d seq=%lu\r\n",
              (int)shell_bool(OPERATE_ID_ONLINE), (int)shell_bool(OPERATE_ID_ENABLE),
              (int)shell_bool(OPERATE_ID_AIM), (int)shell_bool(OPERATE_ID_NAV),
              (int)shell_bool(OPERATE_ID_FRICTION), (int)shell_bool(OPERATE_ID_SHOOT),
              (int)shell_bool(OPERATE_ID_CUSTOM), (unsigned long)seq);
    shell_out("  move=%d %d mm/s spin=%d mrad/s\r\n",
              (int)(shell_float(OPERATE_ID_FORWARD) * 1000.0f),
              (int)(shell_float(OPERATE_ID_RIGHT) * 1000.0f),
              (int)(shell_float(OPERATE_ID_SPIN) * 1000.0f));
    shell_out("  dyaw=%d mrad dpitch=%d mrad\r\n",
              (int)(shell_float(OPERATE_ID_YAW_DELTA) * 1000.0f),
              (int)(shell_float(OPERATE_ID_PITCH_DELTA) * 1000.0f));
    return 0;
}
SHELL_CMD(operate, shell_operate, show_unified_operator_command)
#endif
