#pragma once
#include "IHBridge.hpp"

template<uint8_t instance>
class HBridgeMock: public IHBridge<HBridgeMock<instance>>{
    private:
        using Base = IHBridge<HBridgeDriver<instance>>;
        using Config = typename Base::Config;
        friend class IHBridge<HBridgeMock<instance>>;
        static bool InitImpl(Config &config);
        static bool SetConfigImpl(Config &config);
        
    public:
        template<PhaseState A, PhaseState B, PhaseState C>
        static void Write_Q16(uint16_t dutyCycle);

};