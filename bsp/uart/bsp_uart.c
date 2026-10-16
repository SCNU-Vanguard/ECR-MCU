#include "bsp/uart/bsp_uart.h"
#include "component/common/initcall.h"
#include "component/common/err.h"

static bsp_uart_state_t uart_states[UART_NUM];

static int uart_find(const UART_HandleTypeDef *huart)
{
    for (int uart_index = 0; uart_index < UART_NUM; uart_index++)
    {
        if (board_uart_cfg[uart_index].huart == huart)
        {
            return uart_index;
        }
    }
    return -1;
}

/* RX 是否为循环 DMA：由 CubeMX 里该 DMA 请求的 Mode 决定（Normal / Circular） */
static inline bool rx_circular(const bsp_uart_cfg_t *config)
{
    return config->huart->hdmarx != NULL && config->huart->hdmarx->Init.Mode == DMA_CIRCULAR;
}

/* ---------------- 接收 ---------------- */
static void rx_start(int id)
{
    const bsp_uart_cfg_t *config = &board_uart_cfg[id];
    HAL_StatusTypeDef hal_status;

    uart_states[id].last_position = 0;
    if (config->huart->hdmarx != NULL)
    {
        hal_status = HAL_UARTEx_ReceiveToIdle_DMA(config->huart, config->rx_buffer, config->rx_size);
        /* Normal 模式下把"半满"事件关掉：一次空闲 = 一次回调，不会在报文中间被切开 */
        if (hal_status == HAL_OK && !rx_circular(config))
        {
            __HAL_DMA_DISABLE_IT(config->huart->hdmarx, DMA_IT_HT);
        }
    }
    else
    {
        hal_status = HAL_UARTEx_ReceiveToIdle_IT(config->huart, config->rx_buffer, config->rx_size);   /* 没配 DMA 时退化为中断接收 */
    }
    if (hal_status != HAL_OK)
    {
        uart_states[id].error_count++;
    }
}

static void rx_push(int id, const uint8_t *data, uint16_t length)
{
    bsp_uart_state_t *state = &uart_states[id];

    if (length == 0u)
    {
        return;
    }
    state->rx_bytes += length;
    if (state->event != NULL)
    {
        (void)event_post_isr(state->event, data, length);    /* 拷贝进事件队列，任务里处理 */
    }
    else
    {
        state->rx_unbound_count += length;
    }
}

/* HAL 在"空闲线 / 缓冲半满 / 缓冲满"时调用；size = 缓冲区里目前已有的字节数 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    int id = uart_find(huart);

    if (id < 0)
    {
        return;
    }
    const bsp_uart_cfg_t *config = &board_uart_cfg[id];
    bsp_uart_state_t *state = &uart_states[id];

    if (rx_circular(config))                     /* 循环：按 "上次位置 -> 当前位置" 取增量，可能绕回 */
    {
        if (size > config->rx_size)
        {
            size = config->rx_size;
        }
        if (size >= state->last_position)
        {
            rx_push(id, config->rx_buffer + state->last_position, (uint16_t)(size - state->last_position));
        }
        else
        {
            rx_push(id, config->rx_buffer + state->last_position, (uint16_t)(config->rx_size - state->last_position));
            rx_push(id, config->rx_buffer, size);
        }
        state->last_position = (size == config->rx_size) ? 0u : size;
    }
    else                                         /* Normal / 中断：每次都是从缓冲区头开始的一段，处理完重启 */
    {
        rx_push(id, config->rx_buffer, size);
        rx_start(id);
    }
}

/* ---------------- 发送 ---------------- */
/* 调用时必须已屏蔽中断（os_irq_save）或本身就在串口中断里 */
static void tx_kick(int id)
{
    const bsp_uart_cfg_t *config = &board_uart_cfg[id];
    bsp_uart_state_t *state = &uart_states[id];

    if (state->tx_busy)
    {
        return;
    }
    uint32_t chunk_length = cqueue_out(&state->tx_queue, config->tx_buffer, config->tx_size);

    if (chunk_length == 0u)
    {
        return;
    }
    HAL_StatusTypeDef hal_status = (config->huart->hdmatx != NULL)
        ? HAL_UART_Transmit_DMA(config->huart, config->tx_buffer, (uint16_t)chunk_length)
        : HAL_UART_Transmit_IT(config->huart, config->tx_buffer, (uint16_t)chunk_length);

    if (hal_status != HAL_OK)
    {
        state->tx_drop_count += chunk_length;
        return;
    }
    state->tx_busy = 1u;
    state->tx_bytes += chunk_length;
}

/* 最后一个停止位发完（TC）后 HAL 才调用；DMA 与中断发送都是如此 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    int id = uart_find(huart);

    if (id < 0)
    {
        return;
    }
    uart_states[id].tx_busy = 0u;
    tx_kick(id);                                                 /* 队列里还有就接着发 */
}

/* 溢出/帧错误/奇偶错误：HAL 已中止传输，这里计数并重启 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    int id = uart_find(huart);

    if (id < 0)
    {
        return;
    }
    uart_states[id].error_count++;
    if (huart->gState == HAL_UART_STATE_READY)
    {
        uart_states[id].tx_busy = 0u;
    }
    if (huart->RxState == HAL_UART_STATE_READY && board_uart_cfg[id].rx_size != 0u)
    {
        rx_start(id);
    }
}

/* ---------------- 对外接口 ---------------- */
int32_t bsp_uart_write(uart_id_e id, const void *data, uint32_t len)
{
    if ((unsigned)id >= UART_NUM)
    {
        return ERR_INVAL;
    }
    uint32_t irq_key = os_irq_save();                /* 多个任务可能同时写：入队 + 启动发送整体加锁（微秒级） */
    uint32_t queued_length = cqueue_in(&uart_states[id].tx_queue, data, len);

    tx_kick((int)id);
    os_irq_restore(irq_key);
    return (int32_t)queued_length;
}

void bsp_uart_bind_event(uart_id_e id, struct event_task *task)
{
    if ((unsigned)id < UART_NUM)
    {
        uart_states[id].event = task;
    }
}

const bsp_uart_state_t *bsp_uart_state(uart_id_e id)
{
    return ((unsigned)id < UART_NUM) ? &uart_states[id] : NULL;
}

static int32_t bsp_uart_init(void)
{
    int32_t ret = ERR_OK;

    for (int uart_index = 0; uart_index < UART_NUM; uart_index++)
    {
        const bsp_uart_cfg_t *config = &board_uart_cfg[uart_index];

        if (cqueue_init(&uart_states[uart_index].tx_queue, config->tx_queue_mem, config->tx_queue_size, 1u) != ERR_OK)
        {
            ret = ERR_INVAL;                     /* 队列大小不是 2 的幂：立刻报错，不要静默 */
            continue;
        }
        if (config->rx_size != 0u)
        {
            rx_start(uart_index);
        }
    }
    return ret;
}
INIT_BSP_STAGE1(bsp_uart_init);
