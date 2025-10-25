#pragma once
#include "drivers/system/IPWM.h"
#include "drivers/system/IAdcManager.h"
#include "drivers/sensor/IHallSensor.h"
#include "drivers/sensor/IEncoder.h"
#include "drivers/sensor/IBemfZc.h"
#include "core/motorControl/types.h"

namespace EcoDrive{

struct MotorParameter{
    float polePairs;
    float fluxLinkage_Wb;
    float statorResistance_ohm;
    float statorInductance_H;
    float inertia_kgm2;
    float friction_NmPs;
    float KV_rpmPerVolt;
    float maximumCurrent;
    float maximumVoltage;
};



class PmsmControl{
    static inline PWM pwmDriver;
    static inline AdcManager adcManager;
    static inline MotorParameter motorParams;
    // Position Sensors
    static inline HallSensor* hallSensor;
    static inline IEncoder* encoder;
    static inline BemfZc<Types::BemfZcType::triplePhase> *bemfZcDetector;

    
    public:

    static void init();
    static void update(Types::pwmTicks_t currentPwmTick);
    ~PmsmControl() = default;
};

}