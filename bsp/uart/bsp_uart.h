#ifndef BSP_UART_H__
#define BSP_UART_H__
#include "bsp/include/bsp_hal.h"
#include "component/common/types.h"
#include "component/common/cqueue.h"
#include "osal/osal.h"
#include "osal/event.h"
#include "bsp/include/bsp_ids.h"

/* 只读配置：由 board.c 用 BSP_UART_CFG 生成 */
typedef struct
{
    UART_HandleTypeDef *huart;        /* CubeIDE 生成的句柄，如 &huart1 */
    uint8_t      *rx_buffer;             /* 接收缓冲（DMA 或中断接收直接写入） */
    uint16_t      rx_size;             /* rx_size=0 表示不接收 */
    uint8_t      *tx_buffer;             /* 发送分块缓冲（DMA 直接读） */
    uint16_t      tx_size;
    uint8_t      *tx_queue_mem;            /* 软件发送队列，大小为 2 的幂 */
    uint32_t      tx_queue_size;
    GPIO_TypeDef *direction_port;            /* RS485 方向脚（高 = 发送）；direction_port=NULL 表示没有 */
    uint16_t      direction_pin;
} bsp_uart_cfg_t;

/* 运行状态：全部静态分配 */
typedef struct
{
    struct __cqueue tx_queue;
    volatile uint8_t tx_busy;
    uint16_t last_position;           /* 循环 DMA 模式下上次处理到的位置 */
    struct event_task *event;         /* 接收到的数据块投递给这个事件任务 */
    uint32_t rx_bytes;
    uint32_t tx_bytes;
    uint32_t rx_unbound_count;
    uint32_t tx_drop_count;
    uint32_t error_count;
} bsp_uart_state_t;

/* 每个串口三块缓冲 + 一个配置项。DMA 缓冲放 .dma_nc，软件发送队列放 .sram_bss，都在 AXI SRAM。
 * 接收没有软件队列：每次收到的数据块直接投递给事件任务（bsp_uart_bind_event） */
#define BSP_UART_BUFFERS(_name, _rx_size, _tx_size, _tx_queue_size)                           \
    static uint8_t _name##_rx_buffer[_rx_size]  DMA_NC;                                     \
    static uint8_t _name##_tx_buffer[_tx_size]  DMA_NC;                                     \
    static uint8_t _name##_tx_queue_mem[_tx_queue_size] SRAM_BSS
/* _rx_enable = 0：该口只发不收（rx_size 配 0，初始化时不启动接收） */
#define BSP_UART_CFG(_name, _huart, _rx_enable, _direction_port, _direction_pin)                       \
    {                                                                                    \
        (_huart),                                                                        \
        _name##_rx_buffer,                                                                  \
        (_rx_enable) ? sizeof _name##_rx_buffer : 0u,                                       \
        _name##_tx_buffer,                                                                  \
        sizeof _name##_tx_buffer,                                                           \
        _name##_tx_queue_mem,                                                                 \
        sizeof _name##_tx_queue_mem,                                                          \
        (_direction_port),                                                                      \
        (_direction_pin),                                                                       \
    }

extern const bsp_uart_cfg_t board_uart_cfg[UART_NUM];    /* 由 board.c 提供 */

/* 写入发送队列并启动发送，不阻塞；返回实际入队字节数（队列满则少于 len） */
int32_t  bsp_uart_write(uart_id_e id, const void *data, uint32_t len);
/* 把这个串口收到的数据块投递给事件任务 task（在中断里拷贝，任务里处理）。
 * 一个数据块 = 一次空闲线 / DMA 半满 / DMA 满事件的数据，长度不超过该串口的接收缓冲；
 * task->max_len 必须不小于该串口的接收缓冲，否则超长的块会被丢弃（task->oversize 计数）。
 * 没有绑定时收到的数据被丢弃，计入 rx_unbound_count。 */
void     bsp_uart_bind_event(uart_id_e id, struct event_task *task);
const bsp_uart_state_t *bsp_uart_state(uart_id_e id);    /* 统计计数，调试用 */
#endif
