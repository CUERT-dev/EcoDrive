#pragma once
#include "IHall.hpp"


template<uint8_t instance>
class HallSensor: public IHall<HallSensor<instance>>{
    private:
    friend class IHall<HallSensor<instance>>; // Grant base access
    static uint8_t ReadImpl();
    static bool InitImpl();
};

#include "HalSensor_halBind.tpp"
