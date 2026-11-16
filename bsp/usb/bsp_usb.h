#ifndef BSP_USB_H__
#define BSP_USB_H__
#include "component/common/types.h"

#include "osal/event.h"

/* USB CDC 虚拟串口（与上位机通信）。不阻塞；USB 未枚举时 write 数据留在队列里，满了就丢 */
int32_t  bsp_usb_write(const void *data, uint32_t len);
/* 发送队列当前剩余字节数。write 只入队放得下的部分，要整帧原子发送的调用者先用它判断；
 * 队列只有一个生产者时，查询之后剩余空间只会增大不会减小，所以"够了再写"不会半帧 */
uint32_t bsp_usb_tx_free(void);
/* 把 PC 发来的数据包投递给事件任务 task（在 USB 中断里拷贝，任务里处理）。
 * 全速 CDC 一个包最多 64 字节，所以 task->max_len 至少 64；没有绑定时收到的数据被丢弃 */
void     bsp_usb_bind_event(struct event_task *task);
bool     bsp_usb_connected(void);
#endif
