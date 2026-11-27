#include "component/crc/crc.h"

/* 正向多项式的抽头写在高位侧。低位先入时寄存器右移，抽头要翻到低位侧。 */
static uint32_t reflect_bits(uint32_t value, uint8_t width)
{
    uint32_t reflected = 0u;

    for (uint8_t bit = 0u; bit < width; bit++)
    {
        if ((value & ((uint32_t)1u << bit)) != 0u)
        {
            reflected |= (uint32_t)1u << (width - 1u - bit);
        }
    }
    return reflected;
}

static uint32_t crc_lsb(const uint8_t *data, uint32_t length, uint32_t polynomial, uint32_t initial, uint8_t width)
{
    uint32_t mask = (width == 32u) ? 0xFFFFFFFFu : (((uint32_t)1u << width) - 1u);
    uint32_t crc = initial & mask;
    uint32_t reflected = reflect_bits(polynomial, width) & mask;

    if (data == NULL)
    {
        return crc;
    }
    for (uint32_t index = 0u; index < length; index++)
    {
        crc ^= data[index];
        for (uint8_t bit = 0u; bit < 8u; bit++)
        {
            if ((crc & 1u) != 0u)
            {
                crc = (crc >> 1) ^ reflected;
            }
            else
            {
                crc >>= 1;
            }
        }
        crc &= mask;
    }
    return crc;
}

uint8_t crc8_calculate(const uint8_t *data, uint32_t length, uint8_t polynomial, uint8_t initial)
{
    return (uint8_t)crc_lsb(data, length, polynomial, initial, 8u);
}

uint16_t crc16_calculate(const uint8_t *data, uint32_t length, uint16_t polynomial, uint16_t initial)
{
    return (uint16_t)crc_lsb(data, length, polynomial, initial, 16u);
}

uint32_t crc32_calculate(const uint8_t *data, uint32_t length, uint32_t polynomial, uint32_t initial)
{
    return crc_lsb(data, length, polynomial, initial, 32u);
}

bool crc8_verify(const uint8_t *data, uint32_t length, uint8_t polynomial, uint8_t initial)
{
    if (data == NULL || length < 1u)
    {
        return false;
    }
    return crc8_calculate(data, length - 1u, polynomial, initial) == data[length - 1u];
}

bool crc16_verify(const uint8_t *data, uint32_t length, uint16_t polynomial, uint16_t initial)
{
    uint16_t crc;

    if (data == NULL || length < 2u)
    {
        return false;
    }
    crc = crc16_calculate(data, length - 2u, polynomial, initial);
    return data[length - 2u] == (uint8_t)(crc & 0xFFu) && data[length - 1u] == (uint8_t)(crc >> 8);
}

bool crc32_verify(const uint8_t *data, uint32_t length, uint32_t polynomial, uint32_t initial)
{
    uint32_t crc;

    if (data == NULL || length < 4u)
    {
        return false;
    }
    crc = crc32_calculate(data, length - 4u, polynomial, initial);
    return data[length - 4u] == (uint8_t)(crc & 0xFFu)
        && data[length - 3u] == (uint8_t)((crc >> 8) & 0xFFu)
        && data[length - 2u] == (uint8_t)((crc >> 16) & 0xFFu)
        && data[length - 1u] == (uint8_t)(crc >> 24);
}
