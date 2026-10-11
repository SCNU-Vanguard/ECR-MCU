/* STM32H7 FDCAN（经典 CAN 模式）实现。F4 的 bxCAN 另写 bsp_can_bxcan.c，对上层接口相同。 */
#include "bsp/can/bsp_can.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "osal/osal.h"

typedef struct { uint32_t lo, hi; can_rx_cb_t cb; void *ctx; } sub_t;

static sub_t           subs[CAN_NUM][BSP_CAN_MAX_SUBS];
static uint8_t         nsubs[CAN_NUM];
static bsp_can_stats_t stats[CAN_NUM];

static int can_find(const FDCAN_HandleTypeDef *h)
{
    for (int i = 0; i < CAN_NUM; i++) {
        if (board_can[i] == h) { return i; }
    }
    return -1;
}

int32_t bsp_can_subscribe(can_id_t bus, uint32_t lo, uint32_t hi, can_rx_cb_t cb, void *ctx)
{
    if ((unsigned)bus >= CAN_NUM || cb == NULL || lo > hi) { return ERR_INVAL; }
    uint32_t key = os_irq_save();
    int32_t r = ERR_NOMEM;
    if (nsubs[bus] < BSP_CAN_MAX_SUBS) {
        subs[bus][nsubs[bus]] = (sub_t){ lo, hi, cb, ctx };
        nsubs[bus]++;
        r = ERR_OK;
    }
    os_irq_restore(key);
    return r;
}

/* 订阅回调：把帧拷成 can_event_t 投递给事件任务（ctx 就是事件任务） */
static void post_to_event(can_id_t bus, const can_frame_t *f, void *ctx)
{
    can_event_t e = { .bus = (uint8_t)bus, .len = f->len, .id = (uint16_t)f->id };
    for (uint8_t i = 0; i < f->len; i++) { e.data[i] = f->data[i]; }
    (void)event_post_isr((struct event_task *)ctx, &e, (uint16_t)sizeof e);
}

int32_t bsp_can_subscribe_event(can_id_t bus, uint32_t lo, uint32_t hi, struct event_task *t)
{
    return bsp_can_subscribe(bus, lo, hi, post_to_event, t);
}

int32_t bsp_can_send(can_id_t bus, const can_frame_t *f)
{
    if ((unsigned)bus >= CAN_NUM || f == NULL || f->len > 8u) { return ERR_INVAL; }
    FDCAN_TxHeaderTypeDef h = {
        .Identifier          = f->id,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = ((uint32_t)f->len) << 16,        /* H7 的 DLC 编码在 bit16..19，经典帧 0..8 直接移位 */
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    if (HAL_FDCAN_AddMessageToTxFifoQ(board_can[bus], &h, (uint8_t *)f->data) != HAL_OK) {
        stats[bus].tx_full++;
        return ERR_BUSY;
    }
    stats[bus].tx++;
    return ERR_OK;
}

const bsp_can_stats_t *bsp_can_stats(can_id_t bus)
{
    return ((unsigned)bus < CAN_NUM) ? &stats[bus] : NULL;
}

/* ---- 中断回调（HAL 的弱函数，直接覆盖）---- */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h, uint32_t it)
{
    int bus = can_find(h);
    if (bus < 0 || (it & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) { return; }
    FDCAN_RxHeaderTypeDef rh;
    can_frame_t f;
    while (HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0) > 0u) {      /* 一次中断可能有多帧，取空为止 */
        if (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &rh, f.data) != HAL_OK) { break; }
        f.id  = rh.Identifier;
        f.len = (uint8_t)(rh.DataLength >> 16);
        if (f.len > 8u) { f.len = 8u; }
        stats[bus].rx++;
        bool hit = false;
        for (uint8_t i = 0; i < nsubs[bus]; i++) {
            const sub_t *s = &subs[bus][i];
            if (f.id >= s->lo && f.id <= s->hi) { s->cb((can_id_t)bus, &f, s->ctx); hit = true; }
        }
        if (!hit) { stats[bus].rx_unrouted++; }
    }
}

/* 总线关闭（bus-off）：硬件置 INIT 停止收发。清 INIT 后，控制器检测到 129 x 11 个隐性位才重新入网 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *h, uint32_t flags)
{
    int bus = can_find(h);
    if (bus < 0) { return; }
    if ((flags & FDCAN_IT_BUS_OFF) != 0u) {
        stats[bus].bus_off++;
        CLEAR_BIT(h->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

static int32_t bsp_can_init(void)
{
    int32_t ret = ERR_OK;
    for (int i = 0; i < CAN_NUM; i++) {
        FDCAN_HandleTypeDef *h = board_can[i];
        /* 接收：一条"全收"的范围滤波器 -> FIFO0；其余（含远程帧）一律拒收 */
        FDCAN_FilterTypeDef f = {
            .IdType = FDCAN_STANDARD_ID, .FilterIndex = 0, .FilterType = FDCAN_FILTER_RANGE,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0, .FilterID1 = 0x000, .FilterID2 = 0x7FF,
        };
        /* CubeMX 里该实例的 RxFifo0 元素数为 0 时（本工程的 FDCAN3），硬件没有接收空间 */
        stats[i].rx_enabled = (h->Init.RxFifo0ElmtsNbr != 0u);
        if (stats[i].rx_enabled) {
            if (HAL_FDCAN_ConfigFilter(h, &f) != HAL_OK ||
                HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_REJECT, FDCAN_REJECT,
                                             FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
                HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
                ret = ERR_IO;
            }
        }
        if (HAL_FDCAN_ActivateNotification(h, FDCAN_IT_BUS_OFF, 0) != HAL_OK ||
            HAL_FDCAN_Start(h) != HAL_OK) {
            ret = ERR_IO;
        }
    }
    return ret;
}
INIT_BSP(bsp_can_init);
