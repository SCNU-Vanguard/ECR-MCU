#ifndef BSP_GPIO_H__
#define BSP_GPIO_H__
#include <stdint.h>
#include "bsp/include/bsp_hal.h"
#include "bsp/include/bsp_ids.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} bsp_pin_t;

extern const bsp_pin_t board_pins[GPIO_NUM];       /* 由 board.c 提供 */

static inline void bsp_gpio_write(gpio_id_e id, int level)
{
    HAL_GPIO_WritePin(board_pins[id].port, board_pins[id].pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline int bsp_gpio_read(gpio_id_e id)
{
    return HAL_GPIO_ReadPin(board_pins[id].port, board_pins[id].pin) == GPIO_PIN_SET;
}

static inline void bsp_gpio_toggle(gpio_id_e id)
{
    HAL_GPIO_TogglePin(board_pins[id].port, board_pins[id].pin);
}

/* 外部中断（EXTI）回调：在中断里调用。引脚必须在 CubeMX 里配成外部中断模式并使能对应的 NVIC 中断，
 * 回调才会被调用；触发沿由 CubeMX 的 GPIO 模式决定。
 * 中断优先级必须在 FreeRTOS 允许的范围内（数值 >= configMAX_SYSCALL_INTERRUPT_PRIORITY），回调里才能用 FromISR 类接口。
 * 一个引脚只能绑一个回调，重复绑定返回 ERR_BUSY；回调为 NULL 表示解绑 */
typedef void (*bsp_gpio_exti_callback_t)(void *context);
int32_t bsp_gpio_exti_bind(gpio_id_e id, bsp_gpio_exti_callback_t callback, void *context);
#endif
