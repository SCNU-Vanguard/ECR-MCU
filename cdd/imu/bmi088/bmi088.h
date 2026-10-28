#ifndef CDD_BMI088_H__
#define CDD_BMI088_H__
#include "component/common/types.h"

/* BMI088 驱动：数据就绪（DRDY）中断 + SPI DMA 采集。
 *   - 加速度计 INT1 -> GPIO_ACCEL_INT，陀螺仪 INT3 -> GPIO_GYRO_INT，高电平有效，上升沿触发（CubeMX 里引脚要配成外部中断，上升沿）
 *   - 每次 DRDY 在中断里记录时间戳，并启动一次 SPI DMA 读取（陀螺仪优先，其次加速度计，温度约 4 Hz 一次），整个过程中断里完成，不占任务时间
 *   - 每读完一帧陀螺仪数据，就把"这帧陀螺仪 + 最近一帧加速度计"交给回调（在中断里调用，通常用 event_post_isr 转给任务）
 * 采样率：陀螺仪 2000 Hz（解算频率，每个陀螺仪样本出一帧），加速度计 1600 Hz（只用来校正，用最近一帧）。量程固定：加速度计 ±6 g，陀螺仪 ±2000 dps */
#define BMI088_RATE_HZ                  1600u       /* 陀螺仪 ODR，也是帧率 */
#define BMI088_SAMPLE_PERIOD_US         (1000000u / BMI088_RATE_HZ)
#define BMI088_ACCEL_RATE_HZ            2000u
#define BMI088_ACCEL_SAMPLE_PERIOD_US   (1000000u / BMI088_ACCEL_RATE_HZ)

/* 一帧原始数据（LSB），时间戳是 DRDY 中断里取的 bsp_tick_us 低 32 位（约 71 分钟回绕，做差即可） */
typedef struct
{
    int16_t  gyro_raw[3];
    int16_t  accel_raw[3];
    int16_t  temperature_raw;           /* 原始 11 位补码已展开；还没读到过为 0 */
    uint32_t gyro_timestamp_us;
    uint32_t accel_timestamp_us;
    uint32_t gyro_sequence;             /* 陀螺仪 DRDY 计数，帧之间不连续说明丢了样本 */
    uint32_t accel_sequence;
} bmi088_frame_t;

/* 标定参数（取值见当前兵种的 ins_config.h）：accel = 灵敏度 * raw * scale - scale * offset；gyro = 灵敏度 * raw - offset */
typedef struct
{
    float accel_scale[3];
    float accel_offset[3];              /* 单位 m/s²，与 scale 相乘后扣除 */
    float gyro_offset_rad_s[3];
} bmi088_calibration_t;

/* 换算成物理量之后的数据 */
typedef struct
{
    float gyro_rad_s[3];
    float accel_m_s2[3];
    float temperature_c;
    bool  gyro_clipped;                 /* 任一轴接近量程极限 */
    bool  accel_clipped;
} bmi088_sample_t;

typedef struct
{
    uint32_t gyro_count;                /* 成功读到的陀螺仪帧数 */
    uint32_t accel_count;
    uint32_t temperature_count;
    uint32_t gyro_overrun;              /* 上一次 DRDY 还没来得及读，又来一次（合并成一次读取） */
    uint32_t accel_overrun;
    uint32_t spi_error_count;
    uint32_t start_error_count;         /* 启动 DMA 失败 */
    uint32_t recover_count;
} bmi088_stats_t;

typedef void (*bmi088_frame_callback_t)(const bmi088_frame_t *frame, void *context);

bool    bmi088_present(void);                                       /* 初始化时两颗芯片 ID 都对、寄存器回读一致才为 true */
void    bmi088_get_calibration(bmi088_calibration_t *calibration);   /* 当前生效的标定（来自 ins_config.h） */
void    bmi088_convert(const bmi088_frame_t *frame, bmi088_sample_t *sample);
void    bmi088_set_frame_callback(bmi088_frame_callback_t callback, void *context);     /* 回调在中断里调用 */
int32_t bmi088_start(void);                                         /* 开始采集：登记中断并补读一次，清掉 DRDY 线 */
void    bmi088_get_stats(bmi088_stats_t *stats);
/* 采集中断（太久没有新帧）后在任务里调用：中止在跑的 DMA，重新读一次数据，让 DRDY 线重新产生边沿 */
int32_t bmi088_recover(void);
#endif