#pragma once
#include "driver_common.h"

void usbSerial1_init(const Uart_Config &cfg);
bool usbSerial1_write(const uint8_t* data, uint8_t len);
uint16_t usbSerial1_read(uint8_t* data, uint8_t len);
uint8_t usbSerial1_available();