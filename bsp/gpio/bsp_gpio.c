#include "bsp/gpio/bsp_gpio.h"
#include "component/common/err.h"

typedef struct
{
    bsp_gpio_exti_callback_t callback;
    void *context;
} exti_binding_t;

static exti_binding_t exti_bindings[GPIO_NUM];

int32_t bsp_gpio_exti_bind(gpio_id_e id, bsp_gpio_exti_callback_t callback, void *context)
{
    if ((unsigned)id >= GPIO_NUM)
    {
        return ERR_INVAL;
    }
    if (callback != NULL && exti_bindings[id].callback != NULL)
    {
        return ERR_BUSY;
    }
    exti_bindings[id].context = context;
    exti_bindings[id].callback = callback;
    return ERR_OK;
}

/* HAL 的 EXTI 中断处理最终都调用这个弱符号函数；这里按引脚号找到绑定的回调 */
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    for (int gpio_index = 0; gpio_index < GPIO_NUM; gpio_index++)
    {
        if (board_pins[gpio_index].pin == gpio_pin && exti_bindings[gpio_index].callback != NULL)
        {
            exti_bindings[gpio_index].callback(exti_bindings[gpio_index].context);
        }
    }
}
