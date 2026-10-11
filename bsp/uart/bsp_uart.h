#ifndef BSP_UART_H__
#define BSP_UART_H__
#include "bsp/include/bsp_hal.h"
#include "component/common/types.h"
#include "component/common/cqueue.h"
#include "osal/osal.h"
#include "osal/event.h"
#include "bsp/include/bsp_ids.h"

typedef struct {                      /* 只读配置：由 board.c 用 BSP_UART_CFG 生成 */
    UART_HandleTypeDef *huart;        /* CubeIDE 生成的句柄，如 &huart1 */
    uint8_t  *rx_buf;  uint16_t rx_len;   /* 接收缓冲（DMA 或中断接收直接写入）；rx_len=0 表示不接收 */
    uint8_t  *tx_buf;  uint16_t tx_len;   /* 发送分块缓冲（DMA 直接读） */
    uint8_t  *txq_mem; uint32_t txq_size; /* 软件发送队列，大小为 2 的幂 */
    GPIO_TypeDef *de_port; uint16_t de_pin; /* RS485 方向脚（高 = 发送）；de_port=NULL 表示没有 */
} bsp_uart_cfg_t;

typedef struct {                      /* 运行状态：全部静态分配 */
    struct __cqueue tx_q;
    volatile uint8_t tx_busy;
    uint16_t last_pos;                /* 循环 DMA 模式下上次处理到的位置 */
    struct event_task *ev;            /* 接收到的数据块投递给这个事件任务 */
    uint32_t rx_bytes, tx_bytes, rx_unbound, tx_drop, err_cnt;
} bsp_uart_state_t;

/* 每个串口三块缓冲 + 一个配置项。DMA 缓冲放 .dma_nc（AXI SRAM，不可缓存、DMA 可达）。
 * 接收没有软件队列：每次收到的数据块直接投递给事件任务（bsp_uart_bind_event） */
#define BSP_UART_BUFFERS(_n, _rx, _tx, _txq)                                             \
    static uint8_t _n##_rx_buf[_rx]  DMA_NC;                                             \
    static uint8_t _n##_tx_buf[_tx]  DMA_NC;                                             \
    static uint8_t _n##_txq_mem[_txq]
/* _rx_en = 0：该口只发不收（rx_len 配 0，初始化时不启动接收） */
#define BSP_UART_CFG(_n, _huart, _rx_en, _de_port, _de_pin)                              \
    { (_huart), _n##_rx_buf, (_rx_en) ? sizeof _n##_rx_buf : 0u,                         \
      _n##_tx_buf, sizeof _n##_tx_buf,                                                   \
      _n##_txq_mem, sizeof _n##_txq_mem,                                                 \
      (_de_port), (_de_pin) }

extern const bsp_uart_cfg_t board_uart_cfg[UART_NUM];    /* 由 board.c 提供 */

/* 写入发送队列并启动发送，不阻塞；返回实际入队字节数（队列满则少于 len） */
int32_t  bsp_uart_write(uart_id_t id, const void *data, uint32_t len);
/* 把这个串口收到的数据块投递给事件任务 t（在中断里拷贝，任务里处理）。
 * 一个数据块 = 一次空闲线 / DMA 半满 / DMA 满事件的数据，长度不超过该串口的接收缓冲；
 * t->max_len 必须不小于该串口的接收缓冲，否则超长的块会被丢弃（t->oversize 计数）。
 * 没有绑定时收到的数据被丢弃，计入 rx_unbound。 */
void     bsp_uart_bind_event(uart_id_t id, struct event_task *t);
const bsp_uart_state_t *bsp_uart_state(uart_id_t id);    /* 统计计数，调试用 */
#endif
