#pragma once
#include "drivers/common.h"


void uart1_init(const Uart_Config &cfg);
bool uart1_write(const char *data, uint8_t len);
uint16_t uart1_read(char *data, uint8_t len);
uint8_t uart1_available();

const Uart_Status uart1_getStatus();