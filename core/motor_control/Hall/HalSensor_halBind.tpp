#include "HallSensor.hpp"
#include "Hal/Hal.h"
#include "Config.h"

using namespace Gpio;

using HallAx1 = Pin<DRIVER_Ah_PinNum, DRIVER_Ah_INVERT>;
using HallBx1 = Pin<DRIVER_Bh_PinNum, DRIVER_Bh_INVERT>;
using HallCx1 = Pin<DRIVER_Ch_PinNum, DRIVER_Ch_INVERT>;

/***********************************Hall Effect Sensor HallBinder ****************************/
template <uint8_t instance>
uint8_t HallSensor<instance>::ReadImpl(){
  if constexpr(instance == 0){
    return (HallAx1::Read() << 2) | (HallBx1::Read() << 1) | (HallCx1::Read() << 0);
  }else{
    static_assert(instance == 0,"Only one instance supported for now");
  }
}

template <uint8_t instance>
bool HallSensor<instance>::InitImpl(){
    bool success_flag = true;
    if constexpr(instance == 0){
        HallAx1::SetInput(InputType::PullUp);
        HallBx1::SetInput(InputType::PullUp);
        HallCx1::SetInput(InputType::PullUp);
        Gpio::AttachPCINT(HALL_A_PinNum, &IHall<HallSensor<instance>>::TriggerCallback);
        Gpio::AttachPCINT(HALL_B_PinNum, &IHall<HallSensor<instance>>::TriggerCallback);
        Gpio::AttachPCINT(HALL_C_PinNum, &IHall<HallSensor<instance>>::TriggerCallback);
        HallAx1::EnablePCINT();
        HallBx1::EnablePCINT();
        HallCx1::EnablePCINT();
    }else{
        static_assert(instance == 0,"Only one instance supported for now");
    }
    // Attach to PCINT with lambda
    return success_flag;
}
