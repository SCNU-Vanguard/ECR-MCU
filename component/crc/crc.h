#ifndef CRC_H__
#define CRC_H__
#include "component/common/types.h"

/* 按位计算 CRC。多项式不固定，由调用方传入，库不绑定某一种协议。
 * 只做低位先入：每个字节从 bit0 送入。polynomial 是正向多项式，不含最高次项，
 * 例如 0x31、0x1021、0x04C11DB7。算完不另做异或，协议需要时由调用方处理。
 * calculate：data 为空或 length 为 0 时返回 initial。
 * verify：length 含末尾校验本身。CRC16、CRC32 在帧里低字节在前。
 * data 为空，或长度不够放下校验字节时返回 false。 */
uint8_t  crc8_calculate(const uint8_t *data, uint32_t length, uint8_t polynomial, uint8_t initial);
uint16_t crc16_calculate(const uint8_t *data, uint32_t length, uint16_t polynomial, uint16_t initial);
uint32_t crc32_calculate(const uint8_t *data, uint32_t length, uint32_t polynomial, uint32_t initial);
bool     crc8_verify(const uint8_t *data, uint32_t length, uint8_t polynomial, uint8_t initial);
bool     crc16_verify(const uint8_t *data, uint32_t length, uint16_t polynomial, uint16_t initial);
bool     crc32_verify(const uint8_t *data, uint32_t length, uint32_t polynomial, uint32_t initial);
#endif
