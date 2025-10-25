#pragma once
#include "core/motorControl/types.h"



namespace EcoDrive{

template<Types::BemfZcType VSensor>
class BemfZc{
    public:

    struct Config{
        float hysteresisVoltage_volt;
        float commutationDelay_rad;
    };

    void init(const Config cfg);
    void update(Types::pwmTicks_t currentPwmTick);
    float getElectricalAngle_rad();
    float getElectricalAnglularVelocity_radPs();
};


};