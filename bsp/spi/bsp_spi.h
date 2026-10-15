#ifndef BSP_SPI_H__
#define BSP_SPI_H__
#include "bsp/include/bsp_hal.h"
#include "component/common/types.h"
#include "osal/osal.h"
#include "bsp/include/bsp_ids.h"

typedef struct
{
    SPI_HandleTypeDef *hspi;
    bool use_it;                 /* 该实例的 SPIx_IRQn 已在 CubeMX 里使能（本工程只有 SPI2）；否则只能轮询 */
} bsp_spi_cfg_t;

/* 异步事务完成回调：在中断里调用，result 是 ERR_OK 或 ERR_IO；此时片选已经拉高、总线已经释放，可以在回调里直接发起下一次异步事务 */
typedef void (*bsp_spi_done_callback_t)(int32_t result, void *context);

typedef struct
{
    os_mutex_t lock;
    os_sem_t   done;
    volatile int32_t result;
    volatile bool async_busy;                /* 有异步事务在跑 */
    gpio_id_e async_cs;
    bsp_spi_done_callback_t async_callback;
    void *async_context;
} bsp_spi_state_t;

extern const bsp_spi_cfg_t board_spi_cfg[SPI_NUM];       /* 由 board.c 提供 */

#define BSP_SPI_NO_CS   ((gpio_id_e)-1)
#define BSP_SPI_POLL_MAX  32u    /* 不超过这个长度用轮询（几十微秒内结束），更长的用中断 + 信号量 */

/* 一次完整事务：拉低 cs -> 收发 len 字节 -> 拉高 cs。总线级互斥，任务上下文调用。
 * rx 可为 NULL（只发，SPI6 这种单向口必须这样）；tx 不可为 NULL。
 * 同一条总线不要同时混用同步和异步事务：异步事务在跑时，同步事务返回 ERR_BUSY */
int32_t bsp_spi_xfer(spi_id_e bus, gpio_id_e cs, const uint8_t *tx, uint8_t *rx, uint16_t len);

/* 异步（DMA）事务：拉低 cs，启动 DMA 就返回，完成时在中断里拉高 cs 并调用 done。任务和中断里都能调用，不阻塞。
 *   - 需要在 CubeMX 里给这个 SPI 配好 DMA（RX 和 TX 各一路）并使能 DMA 中断；没配返回 ERR_NODEV
 *   - tx / rx 都必须放在 DMA 能访问的内存（DMA_NC，见 component/common/types.h），否则返回 ERR_INVAL；
 *     DMA_NC 所在的 AXI SRAM 已被 MPU 配成不可缓存，不需要做 Cache 维护
 *   - rx 不能为 NULL（全双工）；len 不超过 65535
 *   - 总线上已经有异步事务在跑返回 ERR_BUSY，调用者自己决定排队还是丢弃 */
int32_t bsp_spi_xfer_async(spi_id_e bus, gpio_id_e cs, const uint8_t *tx, uint8_t *rx, uint16_t len,
                           bsp_spi_done_callback_t done, void *context);
/* 中止在跑的异步事务并拉高片选（超时恢复用）。只能在任务里调用，不触发完成回调 */
int32_t bsp_spi_abort(spi_id_e bus);
#endif
