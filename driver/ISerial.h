#pragma once
#include <stdint.h>
#include "driver/driver_common.h"
#include "platform/driver_core/uart1.h"
#include "platform/driver_core/usbSerial1.h"

enum SerialInstance{
    UART1,
    UART2,
    UART3,
    UART4,
    UART5,
    UART6,
    USBVCOM1,
    USBVCOM2
};

template<uint8_t instance>
class Serial{    
    public:
    
    static void init(const Uart_Config &cfg)
    {
        if constexpr (instance == UART1)return uart1_init(cfg);
        else if constexpr(instance == USBVCOM1)return usbSerial1_init(cfg);
    }

    static bool write(const uint8_t* data , uint8_t len)
    {
        if constexpr (instance == 1)return uart1_write(data , len);
        else if constexpr(instance == USBVCOM1)return usbSerial1_write(data , len);
    }
    
    static uint8_t read(uint8_t* data, uint8_t len)
    {
        if constexpr (instance == 1)return uart1_read(data, len);
    }
    
    static uint8_t available()
    {
        if constexpr (instance == 1)return uart1_available();
    }

    // Optional: Driver status 
    static const Uart_Status getStatus()
    {
        if constexpr (instance == 1)return uart1_getStatus();
    }
};
