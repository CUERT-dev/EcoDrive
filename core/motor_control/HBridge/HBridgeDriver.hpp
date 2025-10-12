#pragma once

#include "IHBridge.hpp"


//Future upgrade for DMA/Interrupt buffer based modulation drivers can be included later


template<uint8_t instance>
class HBridgeDriver: public IHBridge<HBridgeDriver<instance>>{
    private:
        using Base = IHBridge<HBridgeDriver<instance>>;
    
    public:
        using Config = typename Base::Config;
        template<PhaseState A, PhaseState B, PhaseState C>
        static void Write_UQ16(uint16_t dutyCycle);
        static void Write_UQ32(uint32_t dutyCycle);
    
    protected:
        friend class IHBridge<HBridgeDriver<instance>>;
        static bool InitImpl(Config &config);
        static bool SetConfigImpl(Config &config);
};

#include "HBridgeDriver_halBind.tpp"