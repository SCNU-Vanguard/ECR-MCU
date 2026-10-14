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

typedef struct
{
    os_mutex_t lock;
    os_sem_t   done;
    volatile int32_t result;
} bsp_spi_state_t;

extern const bsp_spi_cfg_t board_spi_cfg[SPI_NUM];       /* 由 board.c 提供 */

#define BSP_SPI_NO_CS   ((gpio_id_e)-1)
#define BSP_SPI_POLL_MAX  32u    /* 不超过这个长度用轮询（几十微秒内结束），更长的用中断 + 信号量 */

/* 一次完整事务：拉低 cs -> 收发 len 字节 -> 拉高 cs。总线级互斥，任务上下文调用。
 * rx 可为 NULL（只发，SPI6 这种单向口必须这样）；tx 不可为 NULL。 */
int32_t bsp_spi_xfer(spi_id_e bus, gpio_id_e cs, const uint8_t *tx, uint8_t *rx, uint16_t len);
#endif
