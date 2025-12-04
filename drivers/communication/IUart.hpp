#pragma once
#include <stdint.h>
#include "drivers/stm32f4/uart1.h"
#include "drivers/common.h"


template<uint8_t instance>
class Uart{
    
    public:
    
    constexpr static void init(const Uart_Config &cfg)
    {
        if constexpr (instance == 1)return uart1_init(cfg);
    }
    
    constexpr static bool write(char c)
    {
        if constexpr (instance == 1)return uart1_write(&c, 1);
    }

    constexpr static bool write(const char* data , uint8_t len)
    {
        if constexpr (instance == 1)return uart1_write(data , len);
    }
    
    constexpr static char read()
    {
        char c;
        if constexpr (instance == 1)uart1_read(&c, 1);

        return c;
    }
    constexpr static uint16_t read(char* data, uint8_t len)
    {
        if constexpr (instance == 1)return uart1_read(data, len);
    }
    
    constexpr static uint8_t available()
    {
        if constexpr (instance == 1)return uart1_available();
    }

    // Optional: Driver status 
    constexpr static const Uart_Status getStatus()
    {
        if constexpr (instance == 1)return uart1_getStatus();
    }

    constexpr static void clearErrors();

    // Communication callBacks
    constexpr static void errorCallback();
};
