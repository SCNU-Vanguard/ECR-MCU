/* STM32H7 FDCAN（经典 CAN 模式）实现。F4 的 bxCAN 另写 bsp_can_bxcan.c，对上层接口相同。 */
#include "bsp/can/bsp_can.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "osal/osal.h"

typedef struct
{
    uint32_t    id_low;
    uint32_t    id_high;
    can_rx_cb_t callback;
    void       *context;
} can_subscription_t;

static can_subscription_t subscriptions[CAN_NUM][BSP_CAN_MAX_SUBS];
static uint8_t            subscription_counts[CAN_NUM];
static bsp_can_stats_t    can_stats[CAN_NUM];

static int can_find(const FDCAN_HandleTypeDef *hfdcan)
{
    for (int bus_index = 0; bus_index < CAN_NUM; bus_index++)
    {
        if (board_can[bus_index] == hfdcan)
        {
            return bus_index;
        }
    }
    return -1;
}

int32_t bsp_can_subscribe(can_id_e bus, uint32_t id_low, uint32_t id_high, can_rx_cb_t callback, void *context)
{
    if ((unsigned)bus >= CAN_NUM || callback == NULL || id_low > id_high)
    {
        return ERR_INVAL;
    }
    uint32_t irq_key = os_irq_save();
    int32_t ret = ERR_NOMEM;

    if (subscription_counts[bus] < BSP_CAN_MAX_SUBS)
    {
        can_subscription_t *subscription = &subscriptions[bus][subscription_counts[bus]];

        subscription->id_low = id_low;
        subscription->id_high = id_high;
        subscription->callback = callback;
        subscription->context = context;
        subscription_counts[bus]++;
        ret = ERR_OK;
    }
    os_irq_restore(irq_key);
    return ret;
}

/* 订阅回调：把帧拷成 can_event_t 投递给事件任务（context 就是事件任务） */
static void post_to_event(can_id_e bus, const can_frame_t *frame, void *context)
{
    can_event_t event;

    event.bus = (uint8_t)bus;
    event.len = frame->len;
    event.id = (uint16_t)frame->id;
    for (uint8_t byte_index = 0; byte_index < frame->len; byte_index++)
    {
        event.data[byte_index] = frame->data[byte_index];
    }
    (void)event_post_isr((struct event_task *)context, &event, (uint16_t)sizeof event);
}

int32_t bsp_can_subscribe_event(can_id_e bus, uint32_t id_low, uint32_t id_high, struct event_task *task)
{
    return bsp_can_subscribe(bus, id_low, id_high, post_to_event, task);
}

int32_t bsp_can_send(can_id_e bus, const can_frame_t *frame)
{
    if ((unsigned)bus >= CAN_NUM || frame == NULL || frame->len > 8u)
    {
        return ERR_INVAL;
    }
    FDCAN_TxHeaderTypeDef tx_header =
    {
        .Identifier          = frame->id,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = ((uint32_t)frame->len) << 16,        /* H7 的 DLC 编码在 bit16..19，经典帧 0..8 直接移位 */
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };

    if (HAL_FDCAN_AddMessageToTxFifoQ(board_can[bus], &tx_header, (uint8_t *)frame->data) != HAL_OK)
    {
        can_stats[bus].tx_full++;
        return ERR_BUSY;
    }
    can_stats[bus].tx++;
    return ERR_OK;
}

const bsp_can_stats_t *bsp_can_stats(can_id_e bus)
{
    return ((unsigned)bus < CAN_NUM) ? &can_stats[bus] : NULL;
}

/* ---- 中断回调（HAL 的弱函数，直接覆盖）---- */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupt_flags)
{
    int bus = can_find(hfdcan);

    if (bus < 0 || (interrupt_flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u)
    {
        return;
    }
    FDCAN_RxHeaderTypeDef rx_header;
    can_frame_t frame;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u)       /* 一次中断可能有多帧，取空为止 */
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, frame.data) != HAL_OK)
        {
            break;
        }
        frame.id  = rx_header.Identifier;
        frame.len = (uint8_t)(rx_header.DataLength >> 16);
        if (frame.len > 8u)
        {
            frame.len = 8u;
        }
        can_stats[bus].rx++;
        bool routed = false;

        for (uint8_t sub_index = 0; sub_index < subscription_counts[bus]; sub_index++)
        {
            const can_subscription_t *subscription = &subscriptions[bus][sub_index];

            if (frame.id >= subscription->id_low && frame.id <= subscription->id_high)
            {
                subscription->callback((can_id_e)bus, &frame, subscription->context);
                routed = true;
            }
        }
        if (!routed)
        {
            can_stats[bus].rx_unrouted++;
        }
    }
}

/* 总线关闭（bus-off）：硬件置 INIT 停止收发。清 INIT 后，控制器检测到 129 x 11 个隐性位才重新入网 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t error_flags)
{
    int bus = can_find(hfdcan);

    if (bus < 0)
    {
        return;
    }
    if ((error_flags & FDCAN_IT_BUS_OFF) != 0u)
    {
        can_stats[bus].bus_off++;
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

static int32_t bsp_can_init(void)
{
    int32_t ret = ERR_OK;

    for (int bus_index = 0; bus_index < CAN_NUM; bus_index++)
    {
        FDCAN_HandleTypeDef *hfdcan = board_can[bus_index];
        /* 接收：一条"全收"的范围滤波器 -> FIFO0；其余（含远程帧）一律拒收 */
        FDCAN_FilterTypeDef filter =
        {
            .IdType       = FDCAN_STANDARD_ID,
            .FilterIndex  = 0,
            .FilterType   = FDCAN_FILTER_RANGE,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1    = 0x000,
            .FilterID2    = 0x7FF,
        };

        /* CubeMX 里该实例的 RxFifo0 元素数为 0 时，硬件没有接收空间，不配滤波器 */
        can_stats[bus_index].rx_enabled = (hfdcan->Init.RxFifo0ElmtsNbr != 0u);
        if (can_stats[bus_index].rx_enabled)
        {
            if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK ||
                HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                             FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
                HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
            {
                ret = ERR_IO;
            }
        }
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK ||
            HAL_FDCAN_Start(hfdcan) != HAL_OK)
        {
            ret = ERR_IO;
        }
    }
    return ret;
}
INIT_BSP_STAGE2(bsp_can_init);
