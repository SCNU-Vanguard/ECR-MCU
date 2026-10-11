#ifndef BSP_HAL_H__
#define BSP_HAL_H__
/* 按 MCU 家族选 HAL 总头文件：bsp 里其它文件统一 include 本文件，换 F407 时这里加一个分支 */
#if defined(STM32H723xx)
#include "stm32h7xx_hal.h"
#elif defined(STM32F407xx)
#include "stm32f4xx_hal.h"
#else
#error "unknown MCU: define STM32H723xx or STM32F407xx in build/mcu/<mcu>.mk"
#endif
#endif
