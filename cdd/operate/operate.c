#include <string.h>
#include "cdd/operate/operate.h"
#include "operate_config.h"
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

/* 键盘映射表只给键鼠来源用：DT7 的键鼠和图传键盘。图传遥控的对应关系写死在 from_imgtrans_rc。 */
#define OPERATE_USE_KEYMAP  (OPERATE_MODE != OPERATE_MODE_IMGTRANS_RC)
/* 摇杆、拨轮只有 DT7 和图传遥控有。 */
#define OPERATE_USE_STICK   (OPERATE_MODE != OPERATE_MODE_IMGTRANS_KEY)

#if OPERATE_MODE == OPERATE_MODE_IMGTRANS_KEY
#include <math.h>
#endif

#define STICK_FULL  660.0f

static operate_snap_t published;        /* 只在 operate_task 里写，读用 operate_get_snapshot */
static uint32_t       last_frames;
static uint64_t       frame_us;
static bool           frame_stamp;

/* ---- 键盘映射 ---- */
#if OPERATE_USE_KEYMAP
#ifndef OPERATE_REG_TABLE
#error "operate_config.h must define OPERATE_REG_TABLE"
#endif

typedef struct
{
    uint8_t  id;
    uint8_t  attr;
    uint32_t bind;
} operate_reg_t;

#define OPERATE_REG(id, bind, attr) \
    { (uint8_t)(id), (uint8_t)(attr), (uint32_t)(bind) },
static const operate_reg_t operate_regs[] = {
    OPERATE_REG_TABLE
};
#undef OPERATE_REG

static bool reg_on[OPERATE_IN_COUNT];
static bool reg_raw_prev[OPERATE_IN_COUNT];

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

static bool reg_is_on(unsigned id)
{
    return id < OPERATE_IN_COUNT && reg_on[id];
}

/* 新的一帧才采样键。按住跟着键，切换在上升沿翻转。 */
static void keymap_sample(uint32_t bits)
{
    unsigned i;

    for (i = 0u; i < sizeof operate_regs / sizeof operate_regs[0]; i++)
    {
        const operate_reg_t *reg = &operate_regs[i];
        unsigned id = reg->id;
        bool raw;

        if (id >= OPERATE_IN_COUNT)
        {
            continue;
        }
        raw = reg->bind != 0u && (bits & reg->bind) == reg->bind;
        if (reg->attr == OPERATE_AS_TOGGLE)
        {
            if (raw && !reg_raw_prev[id])
            {
                reg_on[id] = !reg_on[id];
            }
        }
        else
        {
            reg_on[id] = raw;
        }
        reg_raw_prev[id] = raw;
    }
}

static void keymap_clear(void)
{
    memset(reg_on, 0, sizeof reg_on);
    memset(reg_raw_prev, 0, sizeof reg_raw_prev);
}

static void keymap_fill_custom(operate_snap_t *out)
{
    out->custom1 = reg_is_on(OPERATE_IN_CUSTOM1);
    out->custom2 = reg_is_on(OPERATE_IN_CUSTOM2);
    out->custom3 = reg_is_on(OPERATE_IN_CUSTOM3);
    out->custom4 = reg_is_on(OPERATE_IN_CUSTOM4);
}
#endif

/* ---- 摇杆 ---- */
#if OPERATE_USE_STICK
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

/* ---- 公共 ---- */
static void publish(const operate_snap_t *out)
{
    uint32_t irq_key = os_irq_save();

    published = *out;
    os_irq_restore(irq_key);
}

/* 离线时发布全零快照，seq 保留，切换锁存和时间戳清掉。返回 true 表示本周期到此为止。 */
static bool publish_if_offline(bool online)
{
    operate_snap_t out;

    if (online)
    {
        return false;
    }
    memset(&out, 0, sizeof out);
    out.seq = published.seq;
#if OPERATE_USE_KEYMAP
    keymap_clear();
#endif
    frame_stamp = false;
    publish(&out);
    return true;
}

/* 输入帧计数变了才算新帧。 */
static bool take_frame(uint32_t frames)
{
    bool is_new = frames != last_frames;

    last_frames = frames;
    return is_new;
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

/* 新帧才推进 seq。增量是上一帧到这一帧摇杆转过的角，再加上这一帧鼠标。
 * 没有新帧时增量保持上一次，用 seq 判断是否已经消费过。 */
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
    /* 任务被拖住或链路短暂卡顿时，间隔会很长。限幅后只积一个正常步长，避免云台一次猛转。 */
    if (interval > OPERATE_FRAME_INTERVAL_MAX_S)
    {
        interval = OPERATE_FRAME_INTERVAL_MAX_S;
    }
    out->yaw_delta_rad = yaw_rate * interval + mouse_yaw;
    out->pitch_delta_rad = pitch_rate * interval + mouse_pitch;
    out->seq = published.seq + 1u;
}

/* ---- 三种来源，各填一份快照 ---- */
#if OPERATE_MODE == OPERATE_MODE_DT7
static void from_dt7(void)
{
    dt7_data_t in;
    operate_snap_t out;
    bool new_frame;

    dt7_get(&in);
    if (publish_if_offline(in.online))
    {
        return;
    }
    memset(&out, 0, sizeof out);
    out.seq = published.seq;
    out.online = true;
    /* 左开关：上或中为使能，上档同时开自瞄和导航。右开关：上或中为摩擦轮，上为拨弹。 */
    out.enabled = in.switch_left == DT7_SWITCH_UP || in.switch_left == DT7_SWITCH_MID;
    out.use_aim = in.switch_left == DT7_SWITCH_UP;
    out.use_nav = in.switch_left == DT7_SWITCH_UP;
    out.friction = in.switch_right == DT7_SWITCH_UP || in.switch_right == DT7_SWITCH_MID;
    out.shoot = in.switch_right == DT7_SWITCH_UP;
    out.forward_m_s = stick(in.channel[3], OPERATE_MOVE_MAX_M_S);
    out.right_m_s = stick(in.channel[2], OPERATE_MOVE_MAX_M_S);
    out.spin_rad_s = wheel_spin(in.wheel);
    new_frame = take_frame(in.frames);
    if (new_frame)
    {
        keymap_sample(pointer_bits(in.key, in.mouse_left, in.mouse_right, 0u));
    }
    keymap_fill_custom(&out);
    note_frame(&out, new_frame,
               stick(in.channel[0], -OPERATE_YAW_RATE_RAD_S),
               stick(in.channel[1], OPERATE_PITCH_RATE_RAD_S),
               0.0f, 0.0f);
    publish(&out);
}
#elif OPERATE_MODE == OPERATE_MODE_IMGTRANS_RC
/* 图传遥控的对应关系写死：暂停=使能，自定义左=摩擦轮，扳机=拨弹，
 * 挡位 bit0=自瞄 bit1=导航，拨轮=小陀螺，自定义 1..4 见 imgtrans_data_t.custom。 */
static void from_imgtrans_rc(void)
{
    imgtrans_data_t in;
    operate_snap_t out;
    bool new_frame;

    imgtrans_get(&in);
    if (publish_if_offline(in.online))
    {
        return;
    }
    memset(&out, 0, sizeof out);
    out.seq = published.seq;
    out.online = true;
    out.enabled = in.pause != 0u;
    out.friction = in.custom_left != 0u;
    out.shoot = in.trigger != 0u;
    out.use_aim = (in.gear & 1u) != 0u;
    out.use_nav = (in.gear & 2u) != 0u;
    out.custom1 = in.custom[0] != 0u;
    out.custom2 = in.custom[1] != 0u;
    out.custom3 = in.custom[2] != 0u;
    out.custom4 = in.custom[3] != 0u;
    out.forward_m_s = stick(in.channel[3], OPERATE_MOVE_MAX_M_S);
    out.right_m_s = stick(in.channel[2], OPERATE_MOVE_MAX_M_S);
    out.spin_rad_s = wheel_spin(in.wheel);
    new_frame = take_frame(in.frames);
    note_frame(&out, new_frame,
               stick(in.channel[0], -OPERATE_YAW_RATE_RAD_S),
               stick(in.channel[1], OPERATE_PITCH_RATE_RAD_S),
               0.0f, 0.0f);
    publish(&out);
}
#else
/* 图传键盘：全部来自键盘映射表，yaw、pitch 只来自鼠标。WASD 合成的速度限幅在单位圆内。 */
static void from_imgtrans_key(void)
{
    imgtrans_data_t in;
    operate_snap_t out;
    bool new_frame;
    float forward = 0.0f;
    float right = 0.0f;
    float magnitude;

    imgtrans_get(&in);
    if (publish_if_offline(in.online))
    {
        return;
    }
    memset(&out, 0, sizeof out);
    out.seq = published.seq;
    out.online = true;
    new_frame = take_frame(in.frames);
    if (new_frame)
    {
        keymap_sample(pointer_bits(in.key, in.mouse_left, in.mouse_right, in.mouse_middle));
    }
    out.enabled = reg_is_on(OPERATE_IN_ENABLE);
    out.use_aim = reg_is_on(OPERATE_IN_AIM);
    out.use_nav = reg_is_on(OPERATE_IN_NAV);
    out.friction = reg_is_on(OPERATE_IN_FRICTION);
    out.shoot = reg_is_on(OPERATE_IN_SHOOT);
    keymap_fill_custom(&out);

    forward += reg_is_on(OPERATE_IN_FORWARD) ? 1.0f : 0.0f;
    forward -= reg_is_on(OPERATE_IN_BACK) ? 1.0f : 0.0f;
    right += reg_is_on(OPERATE_IN_RIGHT) ? 1.0f : 0.0f;
    right -= reg_is_on(OPERATE_IN_LEFT) ? 1.0f : 0.0f;
    magnitude = sqrtf(right * right + forward * forward);
    if (magnitude > 1.0f)
    {
        right /= magnitude;
        forward /= magnitude;
    }
    out.forward_m_s = forward * OPERATE_MOVE_MAX_M_S;
    out.right_m_s = right * OPERATE_MOVE_MAX_M_S;
    out.spin_rad_s = reg_is_on(OPERATE_IN_SPIN) ? OPERATE_SPIN_RAD_S : 0.0f;
    note_frame(&out, new_frame, 0.0f, 0.0f,
               -(float)in.mouse_x * OPERATE_MOUSE_YAW_RAD_PER_COUNT,
               (float)in.mouse_y * OPERATE_MOUSE_PITCH_RAD_PER_COUNT);
    publish(&out);
}
#endif

static void operate_run(void *argument)
{
    (void)argument;
#if OPERATE_MODE == OPERATE_MODE_DT7
    from_dt7();
#elif OPERATE_MODE == OPERATE_MODE_IMGTRANS_RC
    from_imgtrans_rc();
#else
    from_imgtrans_key();
#endif
}
/* 输入大约 100 Hz。200 Hz 轮询每帧能看到一次，摇杆角用时间戳，不靠这个周期。 */
PERIOD_TASK_DEFINE(operate_task, OS_FREQ_200HZ, OS_PRIORITY_MEDIUM, OS_STACK_SMALL, operate_run, NULL);

int32_t operate_get_snapshot(operate_snap_t *out)
{
    uint32_t irq_key;

    if (out == NULL)
    {
        return ERR_INVAL;
    }
    irq_key = os_irq_save();
    *out = published;
    os_irq_restore(irq_key);
    return ERR_OK;
}

#ifdef CONFIG_COMPONENT_SHELL
#include "component/shell/shell_cmd.h"

static int shell_operate(int argc, char *argv[])
{
    operate_snap_t s;

    (void)argc;
    (void)argv;
    if (operate_get_snapshot(&s) != ERR_OK)
    {
        return -1;
    }
    shell_out("operate: online=%d enabled=%d aim=%d nav=%d fric=%d shoot=%d custom=%d%d%d%d seq=%lu\r\n",
              (int)s.online, (int)s.enabled, (int)s.use_aim, (int)s.use_nav,
              (int)s.friction, (int)s.shoot,
              (int)s.custom1, (int)s.custom2, (int)s.custom3, (int)s.custom4,
              (unsigned long)s.seq);
    shell_out("  move=%d %d mm/s spin=%d mrad/s\r\n",
              (int)(s.forward_m_s * 1000.0f), (int)(s.right_m_s * 1000.0f),
              (int)(s.spin_rad_s * 1000.0f));
    shell_out("  dyaw=%d mrad dpitch=%d mrad\r\n",
              (int)(s.yaw_delta_rad * 1000.0f), (int)(s.pitch_delta_rad * 1000.0f));
    return 0;
}
SHELL_CMD(operate, shell_operate, show_unified_operator_command)
#endif
