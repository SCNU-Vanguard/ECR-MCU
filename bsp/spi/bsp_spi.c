#include "bsp/spi/bsp_spi.h"
#include "bsp/gpio/bsp_gpio.h"
#include "component/common/initcall.h"

static bsp_spi_state_t spi_states[SPI_NUM];

static int spi_find(const SPI_HandleTypeDef *hspi)
{
    for (int spi_index = 0; spi_index < SPI_NUM; spi_index++)
    {
        if (board_spi_cfg[spi_index].hspi == hspi)
        {
            return spi_index;
        }
    }
    return -1;
}

int32_t bsp_spi_xfer(spi_id_e bus, gpio_id_e cs, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if ((unsigned)bus >= SPI_NUM || tx == NULL || len == 0u)
    {
        return ERR_INVAL;
    }
    const bsp_spi_cfg_t *config = &board_spi_cfg[bus];
    bsp_spi_state_t *state = &spi_states[bus];
    int32_t ret = ERR_OK;

    if (os_mutex_lock(&state->lock, 100u) != ERR_OK)
    {
        return ERR_TIMEOUT;
    }
    if (state->async_busy)
    {
        os_mutex_unlock(&state->lock);
        return ERR_BUSY;
    }
    if (cs != BSP_SPI_NO_CS)
    {
        bsp_gpio_write(cs, 0);
    }

    if (len <= BSP_SPI_POLL_MAX || !config->use_it)
    {
        HAL_StatusTypeDef hal_status = (rx != NULL)
            ? HAL_SPI_TransmitReceive(config->hspi, (uint8_t *)tx, rx, len, 10u)
            : HAL_SPI_Transmit(config->hspi, (uint8_t *)tx, len, 10u);

        if (hal_status != HAL_OK)
        {
            ret = ERR_IO;
        }
    }
    else
    {
        state->result = ERR_OK;
        HAL_StatusTypeDef hal_status = (rx != NULL)
            ? HAL_SPI_TransmitReceive_IT(config->hspi, (uint8_t *)tx, rx, len)
            : HAL_SPI_Transmit_IT(config->hspi, (uint8_t *)tx, len);

        if (hal_status != HAL_OK)
        {
            ret = ERR_IO;
        }
        else if (os_sem_take(&state->done, 50u) != ERR_OK)
        {
            (void)HAL_SPI_Abort(config->hspi);
            ret = ERR_TIMEOUT;
        }
        else
        {
            ret = state->result;
        }
    }

    if (cs != BSP_SPI_NO_CS)
    {
        bsp_gpio_write(cs, 1);
    }
    os_mutex_unlock(&state->lock);
    return ret;
}

int32_t bsp_spi_xfer_async(spi_id_e bus, gpio_id_e cs, const uint8_t *tx, uint8_t *rx, uint16_t len,
                           bsp_spi_done_callback_t done, void *context)
{
    if ((unsigned)bus >= SPI_NUM || tx == NULL || rx == NULL || len == 0u)
    {
        return ERR_INVAL;
    }
    const bsp_spi_cfg_t *config = &board_spi_cfg[bus];
    bsp_spi_state_t *state = &spi_states[bus];

    if (config->hspi->hdmatx == NULL || config->hspi->hdmarx == NULL)
    {
        return ERR_NODEV;                                   /* CubeMX 里没给这个 SPI 配 DMA */
    }
    if (!bsp_dma_address_ok(tx) || !bsp_dma_address_ok(rx))
    {
        return ERR_INVAL;                                   /* 缓冲区不在 DMA 能访问的内存里 */
    }

    uint32_t irq_key = os_irq_save();

    if (state->async_busy)
    {
        os_irq_restore(irq_key);
        return ERR_BUSY;
    }
    state->async_busy = true;
    state->async_cs = cs;
    state->async_callback = done;
    state->async_context = context;
    os_irq_restore(irq_key);

    if (cs != BSP_SPI_NO_CS)
    {
        bsp_gpio_write(cs, 0);
    }
    if (HAL_SPI_TransmitReceive_DMA(config->hspi, (uint8_t *)tx, rx, len) != HAL_OK)
    {
        if (cs != BSP_SPI_NO_CS)
        {
            bsp_gpio_write(cs, 1);
        }
        state->async_busy = false;
        return ERR_IO;
    }
    return ERR_OK;
}

int32_t bsp_spi_abort(spi_id_e bus)
{
    if ((unsigned)bus >= SPI_NUM)
    {
        return ERR_INVAL;
    }
    bsp_spi_state_t *state = &spi_states[bus];
    int32_t ret = (HAL_SPI_Abort(board_spi_cfg[bus].hspi) == HAL_OK) ? ERR_OK : ERR_IO;

    if (state->async_cs != BSP_SPI_NO_CS && state->async_busy)
    {
        bsp_gpio_write(state->async_cs, 1);
    }
    state->async_busy = false;
    return ret;
}

static void spi_done(SPI_HandleTypeDef *hspi, int32_t result)
{
    int id = spi_find(hspi);

    if (id < 0)
    {
        return;
    }
    bsp_spi_state_t *state = &spi_states[id];

    if (state->async_busy)
    {
        bsp_spi_done_callback_t callback = state->async_callback;
        void *context = state->async_context;

        if (state->async_cs != BSP_SPI_NO_CS)
        {
            bsp_gpio_write(state->async_cs, 1);
        }
        state->async_busy = false;                          /* 先释放总线，回调里才能接着发起下一次 */
        if (callback != NULL)
        {
            callback(result, context);
        }
        return;
    }
    state->result = result;
    (void)os_sem_give_isr(&state->done);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_done(hspi, ERR_OK);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_done(hspi, ERR_OK);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    spi_done(hspi, ERR_IO);
}

static int32_t bsp_spi_init(void)
{
    int32_t ret = ERR_OK;

    for (int spi_index = 0; spi_index < SPI_NUM; spi_index++)
    {
        spi_states[spi_index].async_cs = BSP_SPI_NO_CS;
        if (os_mutex_init(&spi_states[spi_index].lock) != ERR_OK || os_sem_init(&spi_states[spi_index].done) != ERR_OK)
        {
            ret = ERR_NOMEM;
        }
    }
    return ret;
}
INIT_BSP_STAGE2(bsp_spi_init);
