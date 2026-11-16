#include "bsp/uart/bsp_uart.h"
#include "component/common/initcall.h"
#include "component/common/err.h"

static bsp_uart_state_t uart_states[UART_NUM];

/* 同一个 huart 可以挂两个逻辑口：一个只收，一个只发。HAL 句柄只由 Cube 初始化一次。 */
static int uart_find_rx(const UART_HandleTypeDef *huart)
{
    for (int uart_index = 0; uart_index < UART_NUM; uart_index++)
    {
        if (board_uart_cfg[uart_index].huart == huart && board_uart_cfg[uart_index].rx_size != 0u)
        {
            return uart_index;
        }
    }
    return -1;
}

static int uart_find_tx(const UART_HandleTypeDef *huart)
{
    int fallback = -1;

    for (int uart_index = 0; uart_index < UART_NUM; uart_index++)
    {
        if (board_uart_cfg[uart_index].huart != huart || board_uart_cfg[uart_index].tx_size == 0u)
        {
            continue;
        }
        if (uart_states[uart_index].tx_busy)
        {
            return uart_index;
        }
        fallback = uart_index;
    }
    return fallback;
}

/* RX 是否为循环 DMA：由 CubeMX 里该 DMA 请求的 Mode 决定（Normal / Circular） */
static inline bool rx_circular(const bsp_uart_cfg_t *config)
{
    return config->huart->hdmarx != NULL && config->huart->hdmarx->Init.Mode == DMA_CIRCULAR;
}

/* ---------------- 接收 ---------------- */
static int32_t rx_start(int id)
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
        return ERR_IO;
    }
    return ERR_OK;
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
    int id = uart_find_rx(huart);

    if (id < 0)
    {
        return;
    }
    const bsp_uart_cfg_t *config = &board_uart_cfg[id];
    bsp_uart_state_t *state = &uart_states[id];

    if (rx_circular(config))                     /* 循环：按 "上次位置 -> 当前位置" 取增量，可能绕回 */
    {
        /* 不信任 size：半满事件给的是 RxXferSize/2，空闲事件给的是 HAL 当时算的值，
         * 而且 USART 中断与其 RX DMA 中断同抢占优先级，空闲回调可能先于挂起的半满/传完回调执行。
         * 写指针直接从 DMA 剩余计数算，回调执行顺序不影响结果。 */
        (void)size;
        uint32_t position = (uint32_t)config->rx_size - __HAL_DMA_GET_COUNTER(config->huart->hdmarx);

        if (position >= config->rx_size)         /* 刚好写满、计数器已重装：写指针就是 0 */
        {
            position = 0u;
        }
        if (position == state->last_position)    /* 没有新字节 */
        {
            return;
        }
        if (position > state->last_position)
        {
            rx_push(id, config->rx_buffer + state->last_position, (uint16_t)(position - state->last_position));
        }
        else                                     /* 写指针绕回了缓冲头 */
        {
            rx_push(id, config->rx_buffer + state->last_position, (uint16_t)(config->rx_size - state->last_position));
            rx_push(id, config->rx_buffer, (uint16_t)position);
        }
        state->last_position = (uint16_t)position;
    }
    else                                         /* Normal / 中断：每次都是从缓冲区头开始的一段，处理完重启 */
    {
        rx_push(id, config->rx_buffer, size);
        (void)rx_start(id);                      /* 失败时 rx_start 已计入 error_count */
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
    int id = uart_find_tx(huart);

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
    int rx_id = uart_find_rx(huart);
    int tx_id = uart_find_tx(huart);

    if (rx_id < 0 && tx_id < 0)
    {
        return;
    }
    if (tx_id >= 0)
    {
        uart_states[tx_id].error_count++;
        if (uart_states[tx_id].tx_busy && huart->gState == HAL_UART_STATE_READY)
        {
            /* 发送被 HAL 中止：这一块数据已经从队列取出，丢了要记账，然后继续发队列里剩下的，否则队列会一直卡住 */
            uart_states[tx_id].tx_drop_count += huart->TxXferSize;
            uart_states[tx_id].tx_busy = 0u;
            tx_kick(tx_id);
        }
    }
    if (rx_id >= 0 && rx_id != tx_id)
    {
        uart_states[rx_id].error_count++;
    }
    if (rx_id >= 0 && huart->RxState == HAL_UART_STATE_READY)
    {
        (void)rx_start(rx_id);
    }
}

/* ---------------- 对外接口 ---------------- */
int32_t bsp_uart_write(uart_id_e id, const void *data, uint32_t len)
{
    if ((unsigned)id >= UART_NUM || board_uart_cfg[id].tx_size == 0u)
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

        if (config->tx_size != 0u)
        {
            bool tx_taken = false;

            for (int previous = 0; previous < uart_index; previous++)
            {
                if (board_uart_cfg[previous].huart == config->huart && board_uart_cfg[previous].tx_size != 0u)
                {
                    tx_taken = true;
                    break;
                }
            }
            if (tx_taken || cqueue_init(&uart_states[uart_index].tx_queue, config->tx_queue_mem, config->tx_queue_size, 1u) != ERR_OK)
            {
                ret = ERR_INVAL;                 /* 同一个 huart 只能有一个发送口，队列大小也必须是 2 的幂 */
                continue;
            }
        }
        if (config->rx_size != 0u)
        {
            bool rx_taken = false;

            for (int previous = 0; previous < uart_index; previous++)
            {
                if (board_uart_cfg[previous].huart == config->huart && board_uart_cfg[previous].rx_size != 0u)
                {
                    rx_taken = true;
                    break;
                }
            }
            if (rx_taken)
            {
                ret = ERR_INVAL;                 /* 同一个 huart 只能启动一次接收 */
                continue;
            }
            if (rx_start(uart_index) != ERR_OK)
            {
                ret = ERR_IO;                    /* 接收没启动起来（错误已在 rx_start 里计数） */
            }
        }
    }
    return ret;
}
INIT_BSP_STAGE1(bsp_uart_init);
