#include "HBridgeDriver.hpp"
#include "Hal/Hal.h"
#include "Config.h"


using namespace Gpio;

using Driver_Ah = Pin<DRIVER_Ah_PinNum, DRIVER_Ah_INVERT>;
using Driver_Bh = Pin<DRIVER_Bh_PinNum, DRIVER_Bh_INVERT>;
using Driver_Ch = Pin<DRIVER_Ch_PinNum, DRIVER_Ch_INVERT>;
using Driver_Al = Pin<DRIVER_Al_PinNum, DRIVER_Al_INVERT>;
using Driver_Bl = Pin<DRIVER_Bl_PinNum, DRIVER_Bl_INVERT>;
using Driver_Cl = Pin<DRIVER_Cl_PinNum, DRIVER_Cl_INVERT>;


template <uint8_t instance>
bool HBridgeDriver<instance>::InitImpl(Config &config){
    if constexpr(instance == 0){
        bool success_flag = true;
        TIM1Pwm_init(config.pwmFreq);
        TIM1APwm_init(SPEED_INVERT);

        Driver_Ah::SetOutput(OutputType::PushPull);
        Driver_Bh::SetOutput(OutputType::PushPull);
        Driver_Ch::SetOutput(OutputType::PushPull);
        Driver_Al::SetOutput(OutputType::PushPull);
        Driver_Bl::SetOutput(OutputType::PushPull);
        Driver_Cl::SetOutput(OutputType::PushPull);
        
        Driver_Ah::Write(LOW);
        Driver_Bh::Write(LOW);
        Driver_Ch::Write(LOW);
        Driver_Al::Write(LOW);
        Driver_Bl::Write(LOW);
        Driver_Cl::Write(LOW);
        return success_flag;
    }else{
        static_assert(instance == 0,"Only one instance supported for now");
    }
}

template <uint8_t instance>
inline bool HBridgeDriver<instance>::SetConfigImpl(Config &config)
{
    bool success_flag = true;
    TIM1Pwm_init(config.pwmFreq);
    return success_flag;
}


template <uint8_t instance>
template<PhaseState A, PhaseState B, PhaseState C>
inline void HBridgeDriver<instance>::Write_UQ16(uint16_t dutyCycle)
{
    if constexpr (A == PhaseState::H){
        Driver_Ah::Write(HIGH);
        Driver_Al::Write(LOW);
    }else if constexpr (A == PhaseState::L){
        Driver_Ah::Write(LOW);
        Driver_Al::Write(HIGH);
    }else if constexpr (A == PhaseState::Z){
        Driver_Ah::Write(LOW);
        Driver_Al::Write(LOW);
    }

    if constexpr (B == PhaseState::H){
        Driver_Bh::Write(HIGH);
        Driver_Bl::Write(LOW);
    }else if constexpr (B == PhaseState::L){
        Driver_Bh::Write(LOW);
        Driver_Bl::Write(HIGH);
    }else if constexpr (B == PhaseState::Z){
        Driver_Bh::Write(LOW);
        Driver_Bl::Write(LOW);
    }

    if constexpr (C == PhaseState::H){
        Driver_Ch::Write(HIGH);
        Driver_Cl::Write(LOW);
    }else if constexpr (C == PhaseState::L){
        Driver_Ch::Write(LOW);
        Driver_Cl::Write(HIGH);
    }else if constexpr (C == PhaseState::Z){
        Driver_Ch::Write(LOW);
        Driver_Cl::Write(LOW);
    }

    TIM1Pwm_ScaleToMax(&dutyCycle, 1);
    // Set PWM once (if any phase is active (All PHases share same PWM source check PROTO_V0 hardware reference))
    TIM1APwm_duty(dutyCycle);
}
