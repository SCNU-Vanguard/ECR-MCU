#include "bsp/uart/bsp_uart.h"
#include "component/common/initcall.h"
#include "component/common/err.h"

static bsp_uart_state_t st[UART_NUM];

static int uart_find(const UART_HandleTypeDef *h)
{
    for (int i = 0; i < UART_NUM; i++) {
        if (board_uart_cfg[i].huart == h) { return i; }
    }
    return -1;
}

static inline void de_set(const bsp_uart_cfg_t *c, GPIO_PinState v)
{
    if (c->de_port != NULL) { HAL_GPIO_WritePin(c->de_port, c->de_pin, v); }
}

/* RX 是否为循环 DMA：由 CubeMX 里该 DMA 请求的 Mode 决定（Normal / Circular） */
static inline bool rx_circular(const bsp_uart_cfg_t *c)
{
    return c->huart->hdmarx != NULL && c->huart->hdmarx->Init.Mode == DMA_CIRCULAR;
}

/* ---------------- 接收 ---------------- */
static void rx_start(int id)
{
    const bsp_uart_cfg_t *c = &board_uart_cfg[id];
    HAL_StatusTypeDef r;
    st[id].last_pos = 0;
    if (c->huart->hdmarx != NULL) {
        r = HAL_UARTEx_ReceiveToIdle_DMA(c->huart, c->rx_buf, c->rx_len);
        /* Normal 模式下把"半满"事件关掉：一次空闲 = 一次回调，不会在报文中间被切开 */
        if (r == HAL_OK && !rx_circular(c)) { __HAL_DMA_DISABLE_IT(c->huart->hdmarx, DMA_IT_HT); }
    } else {
        r = HAL_UARTEx_ReceiveToIdle_IT(c->huart, c->rx_buf, c->rx_len);   /* 没配 DMA 时退化为中断接收 */
    }
    if (r != HAL_OK) { st[id].err_cnt++; }
}

static void rx_push(int id, const uint8_t *p, uint16_t n)
{
    bsp_uart_state_t *s = &st[id];
    if (n == 0u) { return; }
    s->rx_bytes += n;
    if (s->ev != NULL) { (void)event_post_isr(s->ev, p, n); }    /* 拷贝进事件队列，任务里处理 */
    else               { s->rx_unbound += n; }
}

/* HAL 在"空闲线 / 缓冲半满 / 缓冲满"时调用；size = 缓冲区里目前已有的字节数 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    int id = uart_find(huart);
    if (id < 0) { return; }
    const bsp_uart_cfg_t *c = &board_uart_cfg[id];
    bsp_uart_state_t *s = &st[id];

    if (rx_circular(c)) {                        /* 循环：按 "上次位置 -> 当前位置" 取增量，可能绕回 */
        if (size > c->rx_len) { size = c->rx_len; }
        if (size >= s->last_pos) {
            rx_push(id, c->rx_buf + s->last_pos, (uint16_t)(size - s->last_pos));
        } else {
            rx_push(id, c->rx_buf + s->last_pos, (uint16_t)(c->rx_len - s->last_pos));
            rx_push(id, c->rx_buf, size);
        }
        s->last_pos = (size == c->rx_len) ? 0u : size;
    } else {                                     /* Normal / 中断：每次都是从缓冲区头开始的一段，处理完重启 */
        rx_push(id, c->rx_buf, size);
        rx_start(id);
    }
}

/* ---------------- 发送 ---------------- */
/* 调用时必须已屏蔽中断（os_irq_save）或本身就在串口中断里 */
static void tx_kick(int id)
{
    const bsp_uart_cfg_t *c = &board_uart_cfg[id];
    bsp_uart_state_t *s = &st[id];
    if (s->tx_busy) { return; }
    uint32_t n = cqueue_out(&s->tx_q, c->tx_buf, c->tx_len);
    if (n == 0u) { return; }
    de_set(c, GPIO_PIN_SET);                                     /* RS485：先拉高方向脚再发 */
    HAL_StatusTypeDef r = (c->huart->hdmatx != NULL)
        ? HAL_UART_Transmit_DMA(c->huart, c->tx_buf, (uint16_t)n)
        : HAL_UART_Transmit_IT(c->huart, c->tx_buf, (uint16_t)n);
    if (r != HAL_OK) { s->tx_drop += n; de_set(c, GPIO_PIN_RESET); return; }
    s->tx_busy = 1u;
    s->tx_bytes += n;
}

/* 最后一个停止位发完（TC）后 HAL 才调用；DMA 与中断发送都是如此 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    int id = uart_find(huart);
    if (id < 0) { return; }
    st[id].tx_busy = 0u;
    tx_kick(id);                                                 /* 队列里还有就接着发，方向脚保持拉高 */
    if (!st[id].tx_busy) { de_set(&board_uart_cfg[id], GPIO_PIN_RESET); }
}

/* 溢出/帧错误/奇偶错误：HAL 已中止传输，这里计数并重启 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    int id = uart_find(huart);
    if (id < 0) { return; }
    st[id].err_cnt++;
    if (huart->gState == HAL_UART_STATE_READY) {
        st[id].tx_busy = 0u;
        de_set(&board_uart_cfg[id], GPIO_PIN_RESET);
    }
    if (huart->RxState == HAL_UART_STATE_READY && board_uart_cfg[id].rx_len != 0u) { rx_start(id); }
}

/* ---------------- 对外接口 ---------------- */
int32_t bsp_uart_write(uart_id_t id, const void *data, uint32_t len)
{
    if ((unsigned)id >= UART_NUM) { return ERR_INVAL; }
    uint32_t key = os_irq_save();                /* 多个任务可能同时写：入队 + 启动发送整体加锁（微秒级） */
    uint32_t n = cqueue_in(&st[id].tx_q, data, len);
    tx_kick((int)id);
    os_irq_restore(key);
    return (int32_t)n;
}

void bsp_uart_bind_event(uart_id_t id, struct event_task *t)
{
    if ((unsigned)id < UART_NUM) { st[id].ev = t; }
}

const bsp_uart_state_t *bsp_uart_state(uart_id_t id)
{
    return ((unsigned)id < UART_NUM) ? &st[id] : NULL;
}

static int32_t bsp_uart_init(void)
{
    int32_t ret = ERR_OK;
    for (int i = 0; i < UART_NUM; i++) {
        const bsp_uart_cfg_t *c = &board_uart_cfg[i];
        if (cqueue_init(&st[i].tx_q, c->txq_mem, c->txq_size, 1u) != ERR_OK) {
            ret = ERR_INVAL;                     /* 队列大小不是 2 的幂：立刻报错，不要静默 */
            continue;
        }
        de_set(c, GPIO_PIN_RESET);
        if (c->rx_len != 0u) { rx_start(i); }
    }
    return ret;
}
INIT_BSP(bsp_uart_init);
