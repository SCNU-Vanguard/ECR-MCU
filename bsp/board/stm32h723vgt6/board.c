/* H723 开发板：把逻辑外设编号 (bsp_ids.h) 映射到 CubeIDE 生成的句柄。
 * 换板子/换 MCU 只改这个文件（F407 板写自己的 board.c）。
 * 每一段都由对应的 CONFIG_BSP_XXX 控制：配置里关掉的外设抽象，这里不引用它的头文件和句柄。
 * 所有配置表都是"一行一个元素"，第一行注释标明每一列的含义。 */
#include "main.h"          /* ACCEL_CS_Pin 等引脚宏 */
#include "component/common/initcall.h"
#include "component/common/err.h"

/* ---- UART：缓冲大小按各自数据量定（队列必须是 2 的幂） ---- */
#ifdef CONFIG_BSP_UART
#include "usart.h"         /* huart1/2/5/7/10 */
#include "bsp/uart/bsp_uart.h"

/*                 接收DMA缓冲  发送分块  发送队列 */
BSP_UART_BUFFERS(referee,       256, 128,  512);
BSP_UART_BUFFERS(interboard,    128, 128,  512);
BSP_UART_BUFFERS(remote,        36,  1,      2);        /* 遥控器只收，发送缓冲占位 */
BSP_UART_BUFFERS(imgtrans,      512, 128,  512);
BSP_UART_BUFFERS(log,           16,  128, 2048);        /* 日志口：主要是发；RX 只给 shell 用 */

const bsp_uart_cfg_t board_uart_cfg[UART_NUM] =
{
    /* 逻辑口                        名称           句柄      接收使能 */
    [UART_IMGTRANS]   = BSP_UART_CFG(imgtrans,      &huart1,  1),            /* USART1 PA9/PA10，921600 */
    [UART_INTERBOARD] = BSP_UART_CFG(interboard,    &huart2,  1),            /* USART2 PD5/PD6，PD4 硬件 DE，921600，收发都走 DMA */
    [UART_REMOTE]     = BSP_UART_CFG(remote,        &huart5,  1),
    [UART_REFEREE]    = BSP_UART_CFG(referee,       &huart7,  1),            /* UART7 PE7/PE8，115200 */
    [UART_LOG]        = BSP_UART_CFG(log,           &huart10, 1),
};
#endif

/* ---- CAN ---- */
#ifdef CONFIG_BSP_CAN
#include "fdcan.h"         /* hfdcan1/2/3 */
#include "bsp/can/bsp_can.h"

FDCAN_HandleTypeDef *const board_can[CAN_NUM] =
{
    /* 逻辑总线        CubeIDE 句柄 */
    [CAN_BUS0] = &hfdcan1,
    [CAN_BUS1] = &hfdcan2,
    [CAN_BUS2] = &hfdcan3,
};
#endif

/* ---- SPI：SPI2 开了中断（IMU），SPI6 没有（WS2812，单向只发） ---- */
#ifdef CONFIG_BSP_SPI
#include "spi.h"           /* hspi2/6 */
#include "bsp/spi/bsp_spi.h"

const bsp_spi_cfg_t board_spi_cfg[SPI_NUM] =
{
    /* 逻辑口      句柄,    中断是否使能 */
    [SPI_IMU] = { &hspi2, true  },
    [SPI_LED] = { &hspi6, false },
};

/* SPI2 的 MISO 是 PC2_C：H7 的 PC2_C 引脚内部有模拟开关，复位后断开，要闭合才能当数字 IO 用。
 * CubeMX 只生成了 PC3 的闭合（片选 PC3_C），PC2 要自己补；重复闭合无副作用。BMI088 初始化在 CDD 阶段，晚于这里 */
static int32_t board_spi_pins_init(void)
{
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);
    return ERR_OK;
}
INIT_BSP_STAGE1(board_spi_pins_init);
#endif

/* ---- PWM ---- */
#ifdef CONFIG_BSP_PWM
#include "tim.h"           /* htim3/12 */
#include "bsp/pwm/bsp_pwm.h"

const bsp_pwm_cfg_t board_pwm_cfg[PWM_NUM] =
{
    /* 逻辑口             定时器句柄, 通道,          定时器时钟(Hz) */
    [PWM_HEATER] = { &htim3,  TIM_CHANNEL_4, 240000000u },   /* PB1  TIM3_CH4  */
    [PWM_BUZZER] = { &htim12, TIM_CHANNEL_2, 240000000u },   /* PB15 TIM12_CH2 */
};
#endif

/* ---- GPIO ---- */
#ifdef CONFIG_BSP_GPIO
#include "bsp/gpio/bsp_gpio.h"

const bsp_pin_t board_pins[GPIO_NUM] =
{
    /* 逻辑引脚             端口,               引脚号 */
    [GPIO_ACCEL_CS] = { ACCEL_CS_GPIO_Port, ACCEL_CS_Pin },
    [GPIO_GYRO_CS]  = { GYRO_CS_GPIO_Port,  GYRO_CS_Pin  },
    [GPIO_ACCEL_INT] = { ACCEL_INT_GPIO_Port, ACCEL_INT_Pin },      /* PE10，EXTI15_10 */
    [GPIO_GYRO_INT]  = { GYRO_INT_GPIO_Port,  GYRO_INT_Pin  },      /* PE12，EXTI15_10 */
};
#endif
