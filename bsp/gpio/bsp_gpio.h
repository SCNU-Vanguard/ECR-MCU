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
#endif
