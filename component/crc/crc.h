#ifndef CRC_H__
#define CRC_H__
#include "component/common/types.h"

/* RoboMaster 裁判系统串口协议和 VT03 图传遥控帧用的 CRC（不是 USB-CDC 等常见的那种）：
 *   CRC8  ：反射多项式 0x8C，初值 0xFF
 *   CRC16 ：反射多项式 0x8408，初值 0xFFFF；放进帧里时低字节在前
 * verify / append 的 length 都包含末尾的校验字节本身（帧头 5 字节 = 4 字节内容 + 1 字节 CRC8；
 * 整帧 = 前面所有字节 + 2 字节 CRC16）。 */
#define CRC8_INITIAL    0xFFu
#define CRC16_INITIAL   0xFFFFu

uint8_t  crc8_calculate(const uint8_t *data, uint32_t length, uint8_t initial);
uint16_t crc16_calculate(const uint8_t *data, uint32_t length, uint16_t initial);
bool     crc8_verify(const uint8_t *data, uint32_t length);
bool     crc16_verify(const uint8_t *data, uint32_t length);
void     crc8_append(uint8_t *data, uint32_t length);
void     crc16_append(uint8_t *data, uint32_t length);
#endif
