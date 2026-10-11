/* H723 开发板：把逻辑外设编号 (bsp_ids.h) 映射到 CubeIDE 生成的句柄。
 * 换板子/换 MCU 只改这个文件（F407 板写自己的 board.c）。
 * 每一段都由对应的 CONFIG_BSP_XXX 控制：配置里关掉的外设抽象，这里不引用它的头文件和句柄。 */
#include "main.h"          /* ACCEL_CS_Pin 等引脚宏 */

/* ---- UART：缓冲大小按各自数据量定（队列必须是 2 的幂） ---- */
#ifdef CONFIG_BSP_UART
#include "usart.h"         /* huart1/2/5/7/10 */
#include "bsp/uart/bsp_uart.h"

/*                 接收DMA缓冲  发送分块  发送队列 */
BSP_UART_BUFFERS(referee,  256, 128,  512);
BSP_UART_BUFFERS(inter,    128, 128,  512);
BSP_UART_BUFFERS(dbus,      36,   1,    2);        /* DBUS 只收，发送缓冲占位 */
BSP_UART_BUFFERS(imgtrans, 512, 128,  512);
BSP_UART_BUFFERS(logu,      16, 128, 2048);        /* 日志口：主要是发；RX 只给 shell 用 */

const bsp_uart_cfg_t board_uart_cfg[UART_NUM] = {
    [UART_REFEREE]    = BSP_UART_CFG(referee,  &huart1,  1, NULL, 0),
    [UART_INTERBOARD] = BSP_UART_CFG(inter,    &huart2,  1, RS485_DE_GPIO_Port, RS485_DE_Pin),
    [UART_DBUS]       = BSP_UART_CFG(dbus,     &huart5,  1, NULL, 0),
    [UART_IMGTRANS]   = BSP_UART_CFG(imgtrans, &huart7,  1, NULL, 0),
#if defined(CONFIG_SHELL)
    [UART_LOG]        = BSP_UART_CFG(logu,     &huart10, 1, NULL, 0),   /* 接 USB-TTL 的 RX：给 shell 输入命令 */
#else
    [UART_LOG]        = BSP_UART_CFG(logu,     &huart10, 0, NULL, 0),   /* 只发不收 */
#endif
};
#endif

/* ---- CAN ---- */
#ifdef CONFIG_BSP_CAN
#include "fdcan.h"         /* hfdcan1/2/3 */
#include "bsp/can/bsp_can.h"

FDCAN_HandleTypeDef *const board_can[CAN_NUM] = {
    [CAN_BUS0] = &hfdcan1, [CAN_BUS1] = &hfdcan2, [CAN_BUS2] = &hfdcan3,
};
#endif

/* ---- SPI：SPI2 开了中断（IMU），SPI6 没有（WS2812，单向只发） ---- */
#ifdef CONFIG_BSP_SPI
#include "spi.h"           /* hspi2/6 */
#include "bsp/spi/bsp_spi.h"

const bsp_spi_cfg_t board_spi_cfg[SPI_NUM] = {
    [SPI_IMU] = { &hspi2, true  },
    [SPI_LED] = { &hspi6, false },
};
#endif

/* ---- PWM ---- */
#ifdef CONFIG_BSP_PWM
#include "tim.h"           /* htim3/12 */
#include "bsp/pwm/bsp_pwm.h"

const bsp_pwm_cfg_t board_pwm_cfg[PWM_NUM] = {
    [PWM_HEATER] = { &htim3,  TIM_CHANNEL_4, 240000000u },   /* PB1  TIM3_CH4  */
    [PWM_BUZZER] = { &htim12, TIM_CHANNEL_2, 240000000u },   /* PB15 TIM12_CH2 */
};
#endif

/* ---- GPIO ---- */
#ifdef CONFIG_BSP_GPIO
#include "bsp/gpio/bsp_gpio.h"

const bsp_pin_t board_pins[GPIO_NUM] = {
    [GPIO_ACCEL_CS] = { ACCEL_CS_GPIO_Port, ACCEL_CS_Pin },
    [GPIO_GYRO_CS]  = { GYRO_CS_GPIO_Port,  GYRO_CS_Pin  },
};
#endif
