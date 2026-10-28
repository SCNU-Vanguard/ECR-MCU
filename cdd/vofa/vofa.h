#ifndef CDD_VOFA_H__
#define CDD_VOFA_H__
#include "component/common/types.h"

/* VOFA+，走 UART_VOFA（USART1 TX，PA9）。图传是另一个逻辑口，只收 PA10。
 * 上位机波特率要和 USART1 一致，现在是 921600。
 * vofa_send：JustFloat，一帧 = count 个 float 小端 + 尾 00 00 80 7F，count 不超过 VOFA_CH_MAX。
 * vofa_send_raw：不加帧头帧尾，按调用方给出的字节原样发送。 */

#define VOFA_CH_MAX  16u

/* 非阻塞。ERR_OK 已入队；ERR_BUSY 队列不足；ERR_INVAL 参数不对 */
int32_t vofa_send(const float *data, uint8_t count);
int32_t vofa_send_raw(const void *data, uint32_t length);
#endif
