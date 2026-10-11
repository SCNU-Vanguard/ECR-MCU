#include "bsp/spi/bsp_spi.h"
#include "bsp/gpio/bsp_gpio.h"
#include "component/common/initcall.h"

static bsp_spi_state_t st[SPI_NUM];

static int spi_find(const SPI_HandleTypeDef *h)
{
    for (int i = 0; i < SPI_NUM; i++) {
        if (board_spi_cfg[i].hspi == h) { return i; }
    }
    return -1;
}

int32_t bsp_spi_xfer(spi_id_t bus, gpio_id_t cs, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if ((unsigned)bus >= SPI_NUM || tx == NULL || len == 0u) { return ERR_INVAL; }
    const bsp_spi_cfg_t *c = &board_spi_cfg[bus];
    bsp_spi_state_t *s = &st[bus];
    int32_t ret = ERR_OK;

    if (os_mutex_lock(&s->lock, 100u) != ERR_OK) { return ERR_TIMEOUT; }
    if (cs != BSP_SPI_NO_CS) { bsp_gpio_write(cs, 0); }

    if (len <= BSP_SPI_POLL_MAX || !c->use_it) {
        HAL_StatusTypeDef r = (rx != NULL)
            ? HAL_SPI_TransmitReceive(c->hspi, (uint8_t *)tx, rx, len, 10u)
            : HAL_SPI_Transmit(c->hspi, (uint8_t *)tx, len, 10u);
        if (r != HAL_OK) { ret = ERR_IO; }
    } else {
        s->result = ERR_OK;
        HAL_StatusTypeDef r = (rx != NULL)
            ? HAL_SPI_TransmitReceive_IT(c->hspi, (uint8_t *)tx, rx, len)
            : HAL_SPI_Transmit_IT(c->hspi, (uint8_t *)tx, len);
        if (r != HAL_OK) {
            ret = ERR_IO;
        } else if (os_sem_take(&s->done, 50u) != ERR_OK) {
            (void)HAL_SPI_Abort(c->hspi);
            ret = ERR_TIMEOUT;
        } else {
            ret = s->result;
        }
    }

    if (cs != BSP_SPI_NO_CS) { bsp_gpio_write(cs, 1); }
    os_mutex_unlock(&s->lock);
    return ret;
}

static void spi_done(SPI_HandleTypeDef *h, int32_t result)
{
    int id = spi_find(h);
    if (id < 0) { return; }
    st[id].result = result;
    (void)os_sem_give_isr(&st[id].done);
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *h)       { spi_done(h, ERR_OK); }
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *h)     { spi_done(h, ERR_OK); }
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *h)        { spi_done(h, ERR_IO); }

static int32_t bsp_spi_init(void)
{
    int32_t ret = ERR_OK;
    for (int i = 0; i < SPI_NUM; i++) {
        if (os_mutex_init(&st[i].lock) != ERR_OK || os_sem_init(&st[i].done) != ERR_OK) { ret = ERR_NOMEM; }
    }
    return ret;
}
INIT_BSP(bsp_spi_init);
