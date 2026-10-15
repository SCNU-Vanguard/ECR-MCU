#include <string.h>
#include "cdd/imu/bmi088/bmi088.h"
#include "cdd/imu/bmi088/bmi088_config.h"
#include "bsp/spi/bsp_spi.h"
#include "bsp/gpio/bsp_gpio.h"
#include "bsp/tick/bsp_tick.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/log/log.h"

/* 寄存器与时序以 BMI088 数据手册为准。SPI：模式 0/3 均可，本板 SPI2 配成模式 3（CPOL=1 CPHA=2edge）。
 *  - 读：地址最高位置 1。加速度计读回的第一个字节是无效字节，陀螺仪没有。
 *  - 加速度计上电后默认 I2C 模式，对它做一次 SPI 读才切到 SPI 模式。 */
#define ACCEL_CHIP_ID           0x00    /* 期望 0x1E */
#define ACCEL_DATA              0x12    /* X_LSB .. Z_MSB，共 6 字节 */
#define ACCEL_TEMPERATURE       0x22    /* MSB、LSB 共 2 字节 */
#define ACCEL_CONF              0x40
#define ACCEL_RANGE             0x41
#define ACCEL_INT1_IO_CTRL      0x53
#define ACCEL_INT_MAP_DATA      0x58
#define ACCEL_PWR_CONF          0x7C
#define ACCEL_PWR_CTRL          0x7D
#define ACCEL_SOFTRESET         0x7E
#define GYRO_CHIP_ID            0x00    /* 期望 0x0F */
#define GYRO_DATA               0x02
#define GYRO_RANGE              0x0F
#define GYRO_BANDWIDTH          0x10
#define GYRO_LPM1               0x11
#define GYRO_SOFTRESET          0x14
#define GYRO_CTRL               0x15
#define GYRO_INT3_INT4_IO_CONF  0x16
#define GYRO_INT3_INT4_IO_MAP   0x18
#define SOFTRESET_CMD           0xB6u

#define ACCEL_CHIP_ID_VALUE     0x1Eu
#define GYRO_CHIP_ID_VALUE      0x0Fu

/* 采样率对应的寄存器取值（改了要和 bmi088.h 的 BMI088_RATE_HZ / BMI088_ACCEL_RATE_HZ 一起改）：
 *   加速度计 ODR 0x0B = 800 Hz（OSR4，高 4 位 0x8）；陀螺仪 0x02 = ODR 1000 Hz / 带宽 116 Hz。
 *   其他档位：加速度计 0x09/0x0A/0x0B/0x0C = 200/400/800/1600 Hz；陀螺仪 0x04/0x03/0x02/0x01 = 200/400/1000/2000 Hz */
#define ACCEL_CONF_VALUE        (0x80u | 0x0Cu)
#define GYRO_BANDWIDTH_VALUE    (0x80u | 0x01u)
#define ACCEL_RANGE_6G          0x01u
#define GYRO_RANGE_2000_DPS     0x00u
/* INT1：使能输出 | 推挽 | 高电平有效；INT3：推挽 | 高电平有效；DRDY 映射到 INT1 / INT3 */
#define ACCEL_INT1_IO_VALUE     ((1u << 3) | (0u << 2) | (1u << 1))
#define ACCEL_INT1_MAP_DRDY     (1u << 2)
#define GYRO_INT3_IO_VALUE      ((0u << 1) | (1u << 0))
#define GYRO_INT3_MAP_DRDY      0x01u
#define GYRO_DRDY_ENABLE        0x80u

/* 灵敏度：加速度计 ±6 g = 32768 LSB（g 取 9.8，与原仓库一致）；陀螺仪 ±2000 dps = 32768 LSB，换算成 rad/s */
#define ACCEL_SENSITIVITY_M_S2_PER_LSB      0.00179443359375f
#define GYRO_SENSITIVITY_RAD_S_PER_LSB      0.00106526443603f
#define TEMPERATURE_C_PER_LSB               0.125f
#define TEMPERATURE_OFFSET_C                23.0f
#define RAW_CLIP_THRESHOLD                  32760       /* 接近 int16 满量程就认为饱和 */

/* 温度只是附带信息，每读这么多帧加速度计就读一次温度（800 Hz / 100 = 8 Hz） */
#define TEMPERATURE_READ_INTERVAL_FRAMES    100u

typedef struct
{
    uint8_t reg;
    uint8_t value;
} register_setting_t;

/* 配置表，一行一个寄存器：{ 寄存器地址, 写入值 }。写完逐个回读校验；顺序按数据手册（先上电再配置，最后开中断） */
static const register_setting_t accel_settings[] =
{
    { ACCEL_PWR_CONF,     0x00u },                  /* 退出挂起 */
    { ACCEL_PWR_CTRL,     0x04u },                  /* 打开加速度计 */
    { ACCEL_CONF,         ACCEL_CONF_VALUE },
    { ACCEL_RANGE,        ACCEL_RANGE_6G },
    { ACCEL_INT1_IO_CTRL, ACCEL_INT1_IO_VALUE },
    { ACCEL_INT_MAP_DATA, ACCEL_INT1_MAP_DRDY },
};

static const register_setting_t gyro_settings[] =
{
    { GYRO_RANGE,           GYRO_RANGE_2000_DPS },
    { GYRO_BANDWIDTH,       GYRO_BANDWIDTH_VALUE },
    { GYRO_LPM1,            0x00u },                /* 正常模式 */
    { GYRO_CTRL,            GYRO_DRDY_ENABLE },
    { GYRO_INT3_INT4_IO_CONF, GYRO_INT3_IO_VALUE },
    { GYRO_INT3_INT4_IO_MAP,  GYRO_INT3_MAP_DRDY },
};

typedef enum
{
    READ_NONE = 0,
    READ_GYRO,
    READ_ACCEL,
    READ_TEMPERATURE
} read_kind_e;

/* DMA 缓冲区放在 DMA_NC（AXI SRAM，不可缓存）：DTCM 里的变量 DMA 访问不到。tx 只有地址有意义，其余字节发 0 */
static uint8_t gyro_tx_buffer[8]    DMA_NC;
static uint8_t gyro_rx_buffer[8]    DMA_NC;
static uint8_t accel_tx_buffer[8]   DMA_NC;
static uint8_t accel_rx_buffer[8]   DMA_NC;
static uint8_t temperature_tx_buffer[4] DMA_NC;
static uint8_t temperature_rx_buffer[4] DMA_NC;

#define GYRO_FRAME_BYTES        7u      /* 地址 + 6 字节数据 */
#define ACCEL_FRAME_BYTES       8u      /* 地址 + 无效字节 + 6 字节数据 */
#define TEMPERATURE_FRAME_BYTES 4u      /* 地址 + 无效字节 + 2 字节数据 */

typedef struct
{
    volatile bool running;
    volatile read_kind_e inflight;
    volatile bool gyro_pending;
    volatile bool accel_pending;
    volatile bool temperature_due;
    volatile uint32_t gyro_drdy_us;
    volatile uint32_t accel_drdy_us;
    volatile uint32_t gyro_sequence;
    volatile uint32_t accel_sequence;
    uint32_t inflight_timestamp_us;
    uint32_t inflight_sequence;
    uint32_t temperature_frame_counter;
    bool accel_valid;
    int16_t accel_raw[3];
    uint32_t accel_timestamp_us;
    uint32_t accel_data_sequence;
    int16_t temperature_raw;
    bmi088_stats_t stats;
    bmi088_frame_callback_t frame_callback;
    void *frame_context;
} bmi088_driver_t;

static bmi088_driver_t driver;
static bool imu_present;
/* 标定表（bmi088_config.h）一行一个轴 -> 拆成三个数组 */
#define CALIBRATION_ACCEL_SCALE(_accel_scale, _accel_offset, _gyro_offset)      (_accel_scale),
#define CALIBRATION_ACCEL_OFFSET(_accel_scale, _accel_offset, _gyro_offset)     (_accel_offset),
#define CALIBRATION_GYRO_OFFSET(_accel_scale, _accel_offset, _gyro_offset)      (_gyro_offset),
static const bmi088_calibration_t calibration =
{
    .accel_scale = { BMI088_CALIBRATION_TABLE(CALIBRATION_ACCEL_SCALE) },
    .accel_offset = { BMI088_CALIBRATION_TABLE(CALIBRATION_ACCEL_OFFSET) },
    .gyro_offset_rad_s = { BMI088_CALIBRATION_TABLE(CALIBRATION_GYRO_OFFSET) },
};

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

/* 写一张配置表并回读校验。初始化阶段调度器还没启动：延时用 HAL_Delay（基于 TIM7 时基），不能用 vTaskDelay */
static int32_t apply_settings(const char *chip_name, const register_setting_t *settings, size_t count, bool is_accel)
{
    for (size_t setting_index = 0; setting_index < count; setting_index++)
    {
        uint8_t read_back = 0u;
        int32_t write_result = is_accel ? accel_write(settings[setting_index].reg, settings[setting_index].value)
                                        : gyro_write(settings[setting_index].reg, settings[setting_index].value);

        HAL_Delay(1);
        int32_t read_result = is_accel ? accel_read(settings[setting_index].reg, &read_back, 1)
                                       : gyro_read(settings[setting_index].reg, &read_back, 1);

        if (write_result != ERR_OK || read_result != ERR_OK || read_back != settings[setting_index].value)
        {
            LOGW("bmi088", "%s reg 0x%02X wrote 0x%02X read 0x%02X", chip_name, settings[setting_index].reg,
                 settings[setting_index].value, read_back);
            return ERR_IO;
        }
        if (is_accel && settings[setting_index].reg == ACCEL_PWR_CTRL)
        {
            HAL_Delay(50);                                  /* 打开加速度计后需要等启动完成 */
        }
    }
    return ERR_OK;
}

static int32_t bmi088_init(void)
{
    uint8_t chip_id = 0;

    imu_present = false;
    memset(&driver, 0, sizeof driver);
    memset(gyro_rx_buffer, 0, sizeof gyro_rx_buffer);          /* .dma_nc 是 NOLOAD 段，上电不清零；DMA 没写到的字节否则会是随机值 */
    memset(accel_rx_buffer, 0, sizeof accel_rx_buffer);
    memset(temperature_rx_buffer, 0, sizeof temperature_rx_buffer);
    gyro_tx_buffer[0] = (uint8_t)(GYRO_DATA | 0x80u);
    accel_tx_buffer[0] = (uint8_t)(ACCEL_DATA | 0x80u);
    temperature_tx_buffer[0] = (uint8_t)(ACCEL_TEMPERATURE | 0x80u);

    (void)accel_read(ACCEL_CHIP_ID, &chip_id, 1);               /* 空读：切到 SPI 模式 */
    (void)accel_write(ACCEL_SOFTRESET, SOFTRESET_CMD);
    HAL_Delay(2);
    (void)accel_read(ACCEL_CHIP_ID, &chip_id, 1);               /* 复位后再空读一次 */
    if (accel_read(ACCEL_CHIP_ID, &chip_id, 1) != ERR_OK || chip_id != ACCEL_CHIP_ID_VALUE)
    {
        LOGW("bmi088", "accel id=0x%02X (want 0x1E)", chip_id);
        return ERR_NODEV;
    }
    (void)gyro_write(GYRO_SOFTRESET, SOFTRESET_CMD);
    HAL_Delay(35);
    if (gyro_read(GYRO_CHIP_ID, &chip_id, 1) != ERR_OK || chip_id != GYRO_CHIP_ID_VALUE)
    {
        LOGW("bmi088", "gyro id=0x%02X (want 0x0F)", chip_id);
        return ERR_NODEV;
    }
    if (apply_settings("accel", accel_settings, sizeof accel_settings / sizeof accel_settings[0], true) != ERR_OK ||
        apply_settings("gyro", gyro_settings, sizeof gyro_settings / sizeof gyro_settings[0], false) != ERR_OK)
    {
        return ERR_IO;
    }
    imu_present = true;
    return ERR_OK;
}
INIT_CDD_STAGE1(bmi088_init);

bool bmi088_present(void)
{
    return imu_present;
}

void bmi088_get_calibration(bmi088_calibration_t *out)
{
    *out = calibration;
}

void bmi088_convert(const bmi088_frame_t *frame, bmi088_sample_t *sample)
{
    sample->gyro_clipped = false;
    sample->accel_clipped = false;
    for (int axis = 0; axis < 3; axis++)
    {
        sample->gyro_rad_s[axis] = GYRO_SENSITIVITY_RAD_S_PER_LSB * (float)frame->gyro_raw[axis] - calibration.gyro_offset_rad_s[axis];
        sample->accel_m_s2[axis] = ACCEL_SENSITIVITY_M_S2_PER_LSB * (float)frame->accel_raw[axis] * calibration.accel_scale[axis] -
                                   calibration.accel_scale[axis] * calibration.accel_offset[axis];
        if (frame->gyro_raw[axis] >= RAW_CLIP_THRESHOLD || frame->gyro_raw[axis] <= -RAW_CLIP_THRESHOLD)
        {
            sample->gyro_clipped = true;
        }
        if (frame->accel_raw[axis] >= RAW_CLIP_THRESHOLD || frame->accel_raw[axis] <= -RAW_CLIP_THRESHOLD)
        {
            sample->accel_clipped = true;
        }
    }
    sample->temperature_c = TEMPERATURE_C_PER_LSB * (float)frame->temperature_raw + TEMPERATURE_OFFSET_C;
}

void bmi088_set_frame_callback(bmi088_frame_callback_t callback, void *context)
{
    driver.frame_context = context;
    driver.frame_callback = callback;
}

void bmi088_get_stats(bmi088_stats_t *stats)
{
    uint32_t irq_state = __get_PRIMASK();

    __disable_irq();
    *stats = driver.stats;
    __set_PRIMASK(irq_state);
}

/* ===== 以下函数都在中断里运行（DRDY 中断、SPI DMA 完成中断；它们在同一个中断优先级，互不抢占）===== */

static void on_spi_done(int32_t result, void *context);

static void start_read(read_kind_e kind)
{
    int32_t ret;

    driver.inflight = kind;
    switch (kind)
    {
    case READ_GYRO:
        driver.gyro_pending = false;
        driver.inflight_timestamp_us = driver.gyro_drdy_us;
        driver.inflight_sequence = driver.gyro_sequence;
        ret = bsp_spi_xfer_async(SPI_IMU, GPIO_GYRO_CS, gyro_tx_buffer, gyro_rx_buffer, GYRO_FRAME_BYTES, on_spi_done, NULL);
        break;
    case READ_ACCEL:
        driver.accel_pending = false;
        driver.inflight_timestamp_us = driver.accel_drdy_us;
        driver.inflight_sequence = driver.accel_sequence;
        ret = bsp_spi_xfer_async(SPI_IMU, GPIO_ACCEL_CS, accel_tx_buffer, accel_rx_buffer, ACCEL_FRAME_BYTES, on_spi_done, NULL);
        break;
    default:
        driver.temperature_due = false;
        ret = bsp_spi_xfer_async(SPI_IMU, GPIO_ACCEL_CS, temperature_tx_buffer, temperature_rx_buffer, TEMPERATURE_FRAME_BYTES,
                                 on_spi_done, NULL);
        break;
    }
    if (ret != ERR_OK)
    {
        driver.inflight = READ_NONE;
        driver.stats.start_error_count++;
    }
}

/* 总线空闲就按优先级启动下一次读取：陀螺仪 > 加速度计 > 温度 */
static void kick(void)
{
    if (!driver.running || driver.inflight != READ_NONE)
    {
        return;
    }
    if (driver.gyro_pending)
    {
        start_read(READ_GYRO);
    }
    else if (driver.accel_pending)
    {
        start_read(READ_ACCEL);
    }
    else if (driver.temperature_due)
    {
        start_read(READ_TEMPERATURE);
    }
}

static void on_gyro_drdy(void *context)
{
    (void)context;
    if (driver.gyro_pending)
    {
        driver.stats.gyro_overrun++;                        /* 上一个样本还没读走，合并，时间戳取最新的 */
    }
    driver.gyro_drdy_us = (uint32_t)bsp_tick_us();
    driver.gyro_sequence++;
    driver.gyro_pending = true;
    kick();
}

static void on_accel_drdy(void *context)
{
    (void)context;
    if (driver.accel_pending)
    {
        driver.stats.accel_overrun++;
    }
    driver.accel_drdy_us = (uint32_t)bsp_tick_us();
    driver.accel_sequence++;
    driver.accel_pending = true;
    kick();
}

static void publish_gyro_frame(void)
{
    bmi088_frame_t frame;

    if (!driver.accel_valid || driver.frame_callback == NULL)
    {
        return;
    }
    for (int axis = 0; axis < 3; axis++)
    {
        frame.gyro_raw[axis] = (int16_t)(gyro_rx_buffer[1 + 2 * axis] | (gyro_rx_buffer[2 + 2 * axis] << 8));   /* 小端：LSB 在前 */
        frame.accel_raw[axis] = driver.accel_raw[axis];
    }
    frame.temperature_raw = driver.temperature_raw;
    frame.gyro_timestamp_us = driver.inflight_timestamp_us;
    frame.accel_timestamp_us = driver.accel_timestamp_us;
    frame.gyro_sequence = driver.inflight_sequence;
    frame.accel_sequence = driver.accel_data_sequence;
    driver.frame_callback(&frame, driver.frame_context);
}

static void on_spi_done(int32_t result, void *context)
{
    read_kind_e kind = driver.inflight;

    (void)context;
    driver.inflight = READ_NONE;
    if (result != ERR_OK)
    {
        driver.stats.spi_error_count++;
        kick();
        return;
    }
    switch (kind)
    {
    case READ_GYRO:
        driver.stats.gyro_count++;
        publish_gyro_frame();
        break;
    case READ_ACCEL:
        for (int axis = 0; axis < 3; axis++)
        {
            driver.accel_raw[axis] = (int16_t)(accel_rx_buffer[2 + 2 * axis] | (accel_rx_buffer[3 + 2 * axis] << 8));
        }
        driver.accel_timestamp_us = driver.inflight_timestamp_us;
        driver.accel_data_sequence = driver.inflight_sequence;
        driver.accel_valid = true;
        driver.stats.accel_count++;
        driver.temperature_frame_counter++;
        if (driver.temperature_frame_counter >= TEMPERATURE_READ_INTERVAL_FRAMES)
        {
            driver.temperature_frame_counter = 0u;
            driver.temperature_due = true;
        }
        break;
    case READ_TEMPERATURE:
    {
        /* 11 位补码：MSB 的 8 位在高，LSB 的高 3 位在低 */
        int32_t raw = ((int32_t)temperature_rx_buffer[2] << 3) | (temperature_rx_buffer[3] >> 5);

        if (raw > 1023)
        {
            raw -= 2048;
        }
        driver.temperature_raw = (int16_t)raw;
        driver.stats.temperature_count++;
        break;
    }
    default:
        break;
    }
    kick();
}

/* ===== 任务上下文 ===== */

int32_t bmi088_start(void)
{
    if (!imu_present)
    {
        return ERR_NODEV;
    }
    int32_t ret = bsp_gpio_exti_bind(GPIO_GYRO_INT, on_gyro_drdy, NULL);

    if (ret == ERR_OK)
    {
        ret = bsp_gpio_exti_bind(GPIO_ACCEL_INT, on_accel_drdy, NULL);
    }
    if (ret != ERR_OK)
    {
        return ret;
    }
    return bmi088_recover();
}

int32_t bmi088_recover(void)
{
    if (!imu_present)
    {
        return ERR_NODEV;
    }
    driver.running = false;
    (void)bsp_spi_abort(SPI_IMU);                           /* 此后不会再有完成回调，inflight 由下面清掉 */

    uint32_t irq_state = __get_PRIMASK();

    __disable_irq();
    driver.inflight = READ_NONE;
    driver.stats.recover_count++;
    driver.gyro_drdy_us = (uint32_t)bsp_tick_us();
    driver.accel_drdy_us = driver.gyro_drdy_us;
    driver.gyro_pending = true;                             /* 不等边沿，直接各读一次：读数据才会拉低 DRDY，之后才有新的上升沿 */
    driver.accel_pending = true;
    driver.running = true;
    kick();
    __set_PRIMASK(irq_state);
    return ERR_OK;
}
