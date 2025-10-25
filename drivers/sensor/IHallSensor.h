#pragma once
#include "core/motorControl/types.h"

namespace EcoDrive{




class HallSensor{
    public:

    struct Config{
        Types::Polarity A_polarity;
        Types::Polarity B_polarity;
        Types::Polarity C_polarity;
        float commutationDelay_rad;
    };

    void init(const Config cfg);
    uint8_t getRawValue();
    void update();
    float getElectricalAngle_rad();
    float getElectricalAnglularVelocity_radPs();



    Types::HallSensorErrorType error();
    
    ~HallSensor() = default;
};


}
