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
#define ACCEL_CHIP_ID   0x00   /* 期望 0x1E */
#define ACCEL_DATA      0x12   /* X_LSB .. Z_MSB，共 6 字节 */
#define ACCEL_PWR_CONF  0x7C
#define ACCEL_PWR_CTRL  0x7D
#define ACCEL_SOFTRESET 0x7E
#define GYRO_CHIP_ID   0x00   /* 期望 0x0F */
#define GYRO_DATA      0x02
#define GYRO_SOFTRESET 0x14
#define SOFTRESET_CMD 0xB6

static bool imu_present;

static int32_t accel_read(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t tx_data[8] = { (uint8_t)(reg | 0x80u) };
    uint8_t rx_data[8];

    if (length > 6u)
    {
        return ERR_INVAL;
    }
    int32_t ret = bsp_spi_xfer(SPI_IMU, GPIO_ACCEL_CS, tx_data, rx_data, (uint16_t)(length + 2u));

    if (ret == ERR_OK)
    {
        memcpy(buffer, &rx_data[2], length);                /* rx_data[0]=地址阶段, rx_data[1]=无效字节 */
    }
    return ret;
}

static int32_t gyro_read(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t tx_data[8] = { (uint8_t)(reg | 0x80u) };
    uint8_t rx_data[8];

    if (length > 6u)
    {
        return ERR_INVAL;
    }
    int32_t ret = bsp_spi_xfer(SPI_IMU, GPIO_GYRO_CS, tx_data, rx_data, (uint16_t)(length + 1u));

    if (ret == ERR_OK)
    {
        memcpy(buffer, &rx_data[1], length);
    }
    return ret;
}

static int32_t accel_write(uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = { (uint8_t)(reg & 0x7Fu), value };

    return bsp_spi_xfer(SPI_IMU, GPIO_ACCEL_CS, tx_data, NULL, 2);
}

static int32_t gyro_write(uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = { (uint8_t)(reg & 0x7Fu), value };

    return bsp_spi_xfer(SPI_IMU, GPIO_GYRO_CS, tx_data, NULL, 2);
}

/* 初始化阶段调度器还没启动：延时用 HAL_Delay（基于 TIM7 时基），不能用 vTaskDelay */
static int32_t bmi088_init(void)
{
    uint8_t chip_id = 0;

    imu_present = false;

    (void)accel_read(ACCEL_CHIP_ID, &chip_id, 1);               /* 空读：切到 SPI 模式 */
    (void)accel_write(ACCEL_SOFTRESET, SOFTRESET_CMD);
    HAL_Delay(2);
    (void)accel_read(ACCEL_CHIP_ID, &chip_id, 1);               /* 复位后再空读一次 */
    if (accel_read(ACCEL_CHIP_ID, &chip_id, 1) != ERR_OK || chip_id != 0x1Eu)
    {
        LOGW("bmi088", "accel id=0x%02X (want 0x1E)", chip_id);
        return ERR_NODEV;
    }
    (void)accel_write(ACCEL_PWR_CONF, 0x00);                    /* 退出挂起 */
    HAL_Delay(1);
    (void)accel_write(ACCEL_PWR_CTRL, 0x04);                    /* 打开加速度计 */
    HAL_Delay(10);

    (void)gyro_write(GYRO_SOFTRESET, SOFTRESET_CMD);
    HAL_Delay(35);
    if (gyro_read(GYRO_CHIP_ID, &chip_id, 1) != ERR_OK || chip_id != 0x0Fu)
    {
        LOGW("bmi088", "gyro id=0x%02X (want 0x0F)", chip_id);
        return ERR_NODEV;
    }
    imu_present = true;                                     /* 量程/ODR 保持上电默认，按需再配 */
    return ERR_OK;
}
INIT_CDD_STAGE1(bmi088_init);

bool bmi088_present(void)
{
    return imu_present;
}

int32_t bmi088_read_raw(bmi088_raw_t *out)
{
    uint8_t accel_bytes[6];
    uint8_t gyro_bytes[6];

    if (!imu_present)
    {
        return ERR_NODEV;
    }
    if (accel_read(ACCEL_DATA, accel_bytes, 6) != ERR_OK || gyro_read(GYRO_DATA, gyro_bytes, 6) != ERR_OK)
    {
        return ERR_IO;
    }
    for (int axis = 0; axis < 3; axis++)
    {
        out->accel[axis]  = (int16_t)(accel_bytes[2 * axis] | (accel_bytes[2 * axis + 1] << 8));    /* 小端：LSB 在前 */
        out->gyro[axis] = (int16_t)(gyro_bytes[2 * axis] | (gyro_bytes[2 * axis + 1] << 8));
    }
    return ERR_OK;
}
