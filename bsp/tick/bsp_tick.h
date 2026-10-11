#ifndef BSP_TICK_H__
#define BSP_TICK_H__
#include <stdint.h>
/* 自启动毫秒数。基于 HAL 时基，中断/任务/调度器启动前都能调用 */
uint32_t bsp_tick_ms(void);
#endif
