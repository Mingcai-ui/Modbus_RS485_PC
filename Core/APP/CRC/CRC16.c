#include "CRC16.h"

uint16_t MODBUS_CRC16(char *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    uint8_t  i;
    char     bit;

    for (i = 0; i < len; i++)
    {
        crc ^= (uint8_t)buf[i];
        for (bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}
