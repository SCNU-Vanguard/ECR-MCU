/* 自测：CAN 收发 + DJI 电机反馈解码，不需要电机。
 *   - 每 10 ms 在 ST_CAN_TX_BUS 上发两帧：
 *       0x123：4 字节递增序号 + 固定校验字节，接收端检查内容和连续性（统计 ok / bad / lost）
 *       0x201：伪造的 M3508 id=1 反馈帧（角度每帧 +50，转速 -123，电流 456，温度 40）
 *   - ST_CAN_RX_BUS 上订阅这两个 ID；0x201 交给真实的 dji_motor 驱动解码，核对多圈累计角度。
 * 允许发送的条件（否则不发，避免在没有应答节点的总线上无限重发）：
 *   Mode 为 Internal/External LoopBack，或配置里 CONFIG_ST_CAN_WIRED = y（两条总线用线连好）。 */
#include "robot/demo/robot_cfg.h"
#include <string.h>
#include "osal/period.h"
#include "osal/event.h"
#include "bsp/can/bsp_can.h"
#include "cdd/motor/dji_motor/dji_motor.h"
#include "component/common/initcall.h"
#include "component/log/log.h"

#ifdef CONFIG_ST_CAN_WIRED
#define ST_WIRED 1
#else
#define ST_WIRED 0
_Static_assert(ST_CAN_TX_BUS == ST_CAN_RX_BUS,
               "loopback self-test needs TX bus == RX bus; set CONFIG_ST_CAN_WIRED = y for two buses");
#endif

#define ID_SEQ     0x123u
#define STEP       50          /* 伪造反馈每帧增加的角度（编码器计数，8192 = 一圈） */

static dji_motor_t st_motor;
static bool        enabled;
static uint32_t    rx_ok, rx_bad, rx_lost, last_seq;      /* 只在 st_seq_task 里读写 */
static bool        seq_started;
static uint32_t    tx_seq;

/* 序号帧也走事件任务：CAN 中断把帧拷成 can_event_t，这里检查内容和序号连续性 */
static void seq_run(void *arg, const uint8_t *msg, uint16_t len)
{
    (void)arg;
    can_event_t e;
    if (msg == NULL || len != sizeof e) { return; }
    memcpy(&e, msg, sizeof e);
    uint32_t s = (uint32_t)e.data[0] | ((uint32_t)e.data[1] << 8) |
                 ((uint32_t)e.data[2] << 16) | ((uint32_t)e.data[3] << 24);
    if (e.len != 8u || e.data[4] != 0xA5u || e.data[5] != 0x5Au ||
        e.data[6] != 0x3Cu || e.data[7] != 0xC3u) { rx_bad++; return; }
    if (seq_started && s != last_seq + 1u) { rx_lost += s - last_seq - 1u; }
    seq_started = true;
    last_seq = s;
    rx_ok++;
}
EVENT_TASK_DEFINE(st_can_seq, 5, 384, EVENT_QUEUE, sizeof(can_event_t), 16, 0, seq_run, NULL);

static int32_t st_can_init(void)
{
    FDCAN_HandleTypeDef *tx = board_can[ST_CAN_TX_BUS];
    bool loop = (tx->Init.Mode == FDCAN_MODE_INTERNAL_LOOPBACK) || (tx->Init.Mode == FDCAN_MODE_EXTERNAL_LOOPBACK);
    enabled = ST_WIRED || loop;
    if (!enabled) {
        LOGW("st_can", "FDCAN mode is NORMAL and CONFIG_ST_CAN_WIRED=n: self-test disabled (set Internal LoopBack in CubeMX)");
        return 0;
    }
    int32_t r = bsp_can_subscribe_event(ST_CAN_RX_BUS, ID_SEQ, ID_SEQ, &st_can_seq);
    if (r != 0) { return r; }
    return dji_motor_register(&st_motor, ST_CAN_RX_BUS, 1, DJI_M3508);   /* 订阅 0x201..0x20B，解码在驱动里 */
}
INIT_APP(st_can_init);

static void can_send_task(void *arg)
{
    (void)arg;
    if (!enabled) { return; }
    can_frame_t f = { .id = ID_SEQ, .len = 8,
                      .data = { (uint8_t)tx_seq, (uint8_t)(tx_seq >> 8), (uint8_t)(tx_seq >> 16), (uint8_t)(tx_seq >> 24),
                                0xA5, 0x5A, 0x3C, 0xC3 } };
    (void)bsp_can_send(ST_CAN_TX_BUS, &f);

    uint16_t ang = (uint16_t)((tx_seq * STEP) % 8192u);
    can_frame_t m = { .id = 0x201, .len = 8,
                      .data = { (uint8_t)(ang >> 8), (uint8_t)ang, 0xFF, 0x85 /* -123 */, 0x01, 0xC8 /* 456 */, 40, 0 } };
    (void)bsp_can_send(ST_CAN_TX_BUS, &m);
    tx_seq++;
}
PERIOD_TASK_DEFINE(st_can_tx, 100, 13, 384, can_send_task, NULL);

static void can_report_task(void *arg)
{
    (void)arg;
    if (!enabled) { return; }
    const bsp_can_stats_t *s = bsp_can_stats(ST_CAN_TX_BUS);
    int32_t expect = (int32_t)(tx_seq * STEP);
    int32_t diff = st_motor.total_angle - expect;
    bool motor_ok = st_motor.got_first && st_motor.speed_rpm == -123 && st_motor.current == 456 &&
                    st_motor.temperature == 40 && diff > -3 * STEP && diff <= 0;
    LOGI("st_can", "tx=%lu seq ok=%lu bad=%lu lost=%lu | motor total=%ld expect=%ld spd=%d cur=%d tmp=%u %s | txfull=%lu busoff=%lu",
         (unsigned long)tx_seq, (unsigned long)rx_ok, (unsigned long)rx_bad, (unsigned long)rx_lost,
         (long)st_motor.total_angle, (long)expect, st_motor.speed_rpm, st_motor.current, st_motor.temperature,
         motor_ok ? "OK" : "CHECK", (unsigned long)s->tx_full, (unsigned long)s->bus_off);
}
PERIOD_TASK_DEFINE(st_can_report, 1, 17, 512, can_report_task, NULL);
