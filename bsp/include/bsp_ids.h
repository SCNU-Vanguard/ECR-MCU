#ifndef BSP_IDS_H__
#define BSP_IDS_H__
/* 逻辑外设编号：按"板上功能"命名，与具体 MCU 的外设号解耦。
 * 每块板的 board.c 负责把它们映射到 CubeIDE 生成的句柄；F407 板以后写自己的 board.c 即可。 */
typedef enum
{
    UART_REFEREE,       /* 裁判系统             */
    UART_INTERBOARD,    /* 双板通信（RS485）    */
    UART_DBUS,          /* DT7 遥控器 DBUS      */
    UART_IMGTRANS,      /* 图传通信             */
    UART_LOG,           /* 日志打印             */
    UART_NUM
} uart_id_e;

/* 具体挂什么电机由兵种配置决定 */
typedef enum
{
    CAN_BUS0,
    CAN_BUS1,
    CAN_BUS2,
    CAN_NUM
} can_id_e;

typedef enum
{
    SPI_IMU,
    SPI_LED,
    SPI_NUM
} spi_id_e;

typedef enum
{
    PWM_HEATER,
    PWM_BUZZER,
    PWM_NUM
} pwm_id_e;

typedef enum
{
    GPIO_ACCEL_CS,
    GPIO_GYRO_CS,
    GPIO_ACCEL_INT,     /* BMI088 加速度计数据就绪（DRDY）中断脚 */
    GPIO_GYRO_INT,      /* BMI088 陀螺仪数据就绪（DRDY）中断脚   */
    GPIO_NUM
} gpio_id_e;
#endif
