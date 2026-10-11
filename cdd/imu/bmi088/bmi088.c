#include <string.h>
#include "cdd/imu/bmi088/bmi088.h"
#include "bsp/spi/bsp_spi.h"
#include "bsp/gpio/bsp_gpio.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/log/log.h"

/* 寄存器与时序以 BMI088 数据手册为准。SPI：模式 0/3 均可，本板 SPI2 配成模式 3（CPOL=1 CPHA=2edge）。
 *  - 读：地址最高位置 1。加速度计读回的第一个字节是无效字节，陀螺仪没有。
 *  - 加速度计上电后默认 I2C 模式，对它做一次 SPI 读才切到 SPI 模式。 */
#define ACC_CHIP_ID   0x00   /* 期望 0x1E */
#define ACC_DATA      0x12   /* X_LSB .. Z_MSB，共 6 字节 */
#define ACC_PWR_CONF  0x7C
#define ACC_PWR_CTRL  0x7D
#define ACC_SOFTRESET 0x7E
#define GYR_CHIP_ID   0x00   /* 期望 0x0F */
#define GYR_DATA      0x02
#define GYR_SOFTRESET 0x14
#define SOFTRESET_CMD 0xB6

static bool present;

static int32_t acc_read(uint8_t reg, uint8_t *buf, uint8_t n)
{
    uint8_t tx[8] = { (uint8_t)(reg | 0x80u) }, rx[8];
    if (n > 6u) { return ERR_INVAL; }
    int32_t r = bsp_spi_xfer(SPI_IMU, GPIO_ACCEL_CS, tx, rx, (uint16_t)(n + 2u));
    if (r == ERR_OK) { memcpy(buf, &rx[2], n); }            /* rx[0]=地址阶段, rx[1]=无效字节 */
    return r;
}
static int32_t gyr_read(uint8_t reg, uint8_t *buf, uint8_t n)
{
    uint8_t tx[8] = { (uint8_t)(reg | 0x80u) }, rx[8];
    if (n > 6u) { return ERR_INVAL; }
    int32_t r = bsp_spi_xfer(SPI_IMU, GPIO_GYRO_CS, tx, rx, (uint16_t)(n + 1u));
    if (r == ERR_OK) { memcpy(buf, &rx[1], n); }
    return r;
}
static int32_t acc_write(uint8_t reg, uint8_t v)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), v };
    return bsp_spi_xfer(SPI_IMU, GPIO_ACCEL_CS, tx, NULL, 2);
}
static int32_t gyr_write(uint8_t reg, uint8_t v)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), v };
    return bsp_spi_xfer(SPI_IMU, GPIO_GYRO_CS, tx, NULL, 2);
}

/* 初始化阶段调度器还没启动：延时用 HAL_Delay（基于 TIM7 时基），不能用 vTaskDelay */
static int32_t bmi088_init(void)
{
    uint8_t id = 0;
    present = false;

    (void)acc_read(ACC_CHIP_ID, &id, 1);                    /* 空读：切到 SPI 模式 */
    (void)acc_write(ACC_SOFTRESET, SOFTRESET_CMD);
    HAL_Delay(2);
    (void)acc_read(ACC_CHIP_ID, &id, 1);                    /* 复位后再空读一次 */
    if (acc_read(ACC_CHIP_ID, &id, 1) != ERR_OK || id != 0x1Eu) {
        LOGW("bmi088", "accel id=0x%02X (want 0x1E)", id);
        return ERR_NODEV;
    }
    (void)acc_write(ACC_PWR_CONF, 0x00);                    /* 退出挂起 */
    HAL_Delay(1);
    (void)acc_write(ACC_PWR_CTRL, 0x04);                    /* 打开加速度计 */
    HAL_Delay(10);

    (void)gyr_write(GYR_SOFTRESET, SOFTRESET_CMD);
    HAL_Delay(35);
    if (gyr_read(GYR_CHIP_ID, &id, 1) != ERR_OK || id != 0x0Fu) {
        LOGW("bmi088", "gyro id=0x%02X (want 0x0F)", id);
        return ERR_NODEV;
    }
    present = true;                                          /* 量程/ODR 保持上电默认，按需再配 */
    return ERR_OK;
}
INIT_CDD(bmi088_init);

bool bmi088_present(void) { return present; }

int32_t bmi088_read_raw(bmi088_raw_t *o)
{
    uint8_t a[6], g[6];
    if (!present) { return ERR_NODEV; }
    if (acc_read(ACC_DATA, a, 6) != ERR_OK || gyr_read(GYR_DATA, g, 6) != ERR_OK) { return ERR_IO; }
    for (int i = 0; i < 3; i++) {
        o->acc[i]  = (int16_t)(a[2 * i] | (a[2 * i + 1] << 8));    /* 小端：LSB 在前 */
        o->gyro[i] = (int16_t)(g[2 * i] | (g[2 * i + 1] << 8));
    }
    return ERR_OK;
}
