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

/* 每个方向一块缓冲。DMA 缓冲和软件发送队列都放 RAM_D1（不可缓存）。
 * 接收没有软件队列：每次收到的数据块直接投递给事件任务（bsp_uart_bind_event）。
 * 尺寸写 0 的方向不分配缓冲。 */
#define BSP_UART_IS_ZERO(x)        BSP_UART_IS_ZERO_I(BSP_UART_ZERO_##x)
#define BSP_UART_IS_ZERO_I(x)      BSP_UART_IS_ZERO_II(x, 0, )
#define BSP_UART_IS_ZERO_II(a, b, ...) b
#define BSP_UART_ZERO_0            ~, 1
#define BSP_UART_RX_DECL_0(_name, _rx_size) \
    static uint8_t _name##_rx_buffer[_rx_size] SRAM_NONCACHEABLE
#define BSP_UART_RX_DECL_1(_name, _rx_size)
#define BSP_UART_RX_DECL(_zero, _name, _rx_size) BSP_UART_RX_DECL_I(_zero, _name, _rx_size)
#define BSP_UART_RX_DECL_I(_zero, _name, _rx_size) BSP_UART_RX_DECL_##_zero(_name, _rx_size)
#define BSP_UART_TX_DECL_0(_name, _tx_size, _tx_queue_size) \
    static uint8_t _name##_tx_buffer[_tx_size] SRAM_NONCACHEABLE; \
    static uint8_t _name##_tx_queue_mem[_tx_queue_size] SRAM_NONCACHEABLE
#define BSP_UART_TX_DECL_1(_name, _tx_size, _tx_queue_size)
#define BSP_UART_TX_DECL(_zero, _name, _tx_size, _tx_queue_size) \
    BSP_UART_TX_DECL_I(_zero, _name, _tx_size, _tx_queue_size)
#define BSP_UART_TX_DECL_I(_zero, _name, _tx_size, _tx_queue_size) \
    BSP_UART_TX_DECL_##_zero(_name, _tx_size, _tx_queue_size)
#define BSP_UART_BUFFERS(_name, _rx_size, _tx_size, _tx_queue_size) \
    BSP_UART_RX_DECL(BSP_UART_IS_ZERO(_rx_size), _name, _rx_size); \
    BSP_UART_TX_DECL(BSP_UART_IS_ZERO(_tx_size), _name, _tx_size, _tx_queue_size)
/* 后两个参数是接收使能、发送使能，只能写 0 或 1。关着的方向 size 为 0，初始化时不启动。 */
#define BSP_UART_CFG(_name, _huart, _rx_enable, _tx_enable)                                   \
    BSP_UART_CFG_##_rx_enable##_##_tx_enable(_name, _huart)
#define BSP_UART_CFG_1_1(_name, _huart)                                                       \
    {                                                                                         \
        (_huart),                                                                             \
        _name##_rx_buffer, sizeof _name##_rx_buffer,                                          \
        _name##_tx_buffer, sizeof _name##_tx_buffer,                                          \
        _name##_tx_queue_mem, sizeof _name##_tx_queue_mem,                                    \
    }
#define BSP_UART_CFG_1_0(_name, _huart)                                                       \
    {                                                                                         \
        (_huart),                                                                             \
        _name##_rx_buffer, sizeof _name##_rx_buffer,                                          \
        NULL, 0u, NULL, 0u,                                                                   \
    }
#define BSP_UART_CFG_0_1(_name, _huart)                                                       \
    {                                                                                         \
        (_huart),                                                                             \
        NULL, 0u,                                                                             \
        _name##_tx_buffer, sizeof _name##_tx_buffer,                                          \
        _name##_tx_queue_mem, sizeof _name##_tx_queue_mem,                                    \
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
