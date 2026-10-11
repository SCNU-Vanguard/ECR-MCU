#ifndef BSP_USB_H__
#define BSP_USB_H__
#include "component/common/types.h"

#include "osal/event.h"

/* USB CDC 虚拟串口（与上位机通信）。不阻塞；USB 未枚举时 write 数据留在队列里，满了就丢 */
int32_t  bsp_usb_write(const void *data, uint32_t len);
/* 把 PC 发来的数据包投递给事件任务 t（在 USB 中断里拷贝，任务里处理）。
 * 全速 CDC 一个包最多 64 字节，所以 t->max_len 至少 64；没有绑定时收到的数据被丢弃 */
void     bsp_usb_bind_event(struct event_task *t);
bool     bsp_usb_connected(void);

/* 下面两个由 CubeIDE 生成的 usbd_cdc_if.c 在 USER CODE 段里调用（在 USB 中断上下文） */
void bsp_usb_rx_isr(const uint8_t *buf, uint32_t len);
void bsp_usb_tx_done_isr(void);
#endif
