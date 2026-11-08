#ifndef __CRC16_H__
#define __CRC16_H__

#include "main.h"

/* buf 声明为 const：CRC 只读不改，这样传 const 指针进来不会丢限定符 */
uint16_t MODBUS_CRC16(const char *buf, uint8_t len);

#endif
