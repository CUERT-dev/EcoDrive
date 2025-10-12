#pragma once
#include <stdint.h>



enum class PhaseState: uint8_t{
    H = 0b10,
    L = 0b01,
    Z = 0b00
};

template<typename Derived>
class IHBridge{
protected:
    IHBridge() = default;
    ~IHBridge() = default; // Prevent slicing

public:
    // Common interface
    struct Config{
        uint16_t pwmFreq = 10000;
        /// Other stuff to be added later (example configurable deadtime duration)
    };
    
    static bool Init(Config &config)
    { 
        return Derived::InitImpl(config); 
    }

    static bool SetConfig(Config &config)
    {
        return Derived::SetConfigImpl(config);
    }
};
