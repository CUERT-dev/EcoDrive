#include "pmsmControl.h"

namespace EcoDrive{

void PmsmControl::init()
{
    // Initialize PWM
    pwmDriver.init({
        .AH_polarity = Types::Polarity::ActiveHigh,
        .BH_polarity = Types::Polarity::ActiveHigh,
        .CH_polarity = Types::Polarity::ActiveHigh,
        .AL_polarity = Types::Polarity::ActiveLow,
        .BL_polarity = Types::Polarity::ActiveLow,
        .CL_polarity = Types::Polarity::ActiveLow,
        .freq_hz = 10000.0f,
        .deadtime_ns = 1500.0f,
        .dutyMax_perc = 95.0f,
        .dutyMin_perc = 10.0f,
        .pwmAlignment = Types::PWMAlignmentType::centerAligned,
        .modulation = Types::PWMModulationType::SVM,
        .breakEnabled = true
    });
    // Initialize ADC Manager
    adcManager.init({
        .voltageSensorType = Types::VoltageSensorType::resistiveDivider,
        .currentSensorType = Types::CurrentSensorType::directInline,
        .samplePwmEdgeDelay_ns = 1000,
        .adcVoltageRefType = Types::InternalAdcReferenceType::bandgap1V2,
        .adcVoltageRef_volt = 1.2
    });
    motorParams = {
        .polePairs = 7,
        .fluxLinkage_Wb = 0.015f,
        .statorResistance_ohm = 0.05f,
        .statorInductance_H = 0.0001f,
        .inertia_kgm2 = 0.0001f,
        .friction_NmPs = 0.0001f,
        .KV_rpmPerVolt = 120.0f,
        .maximumCurrent = 30.0f,
        .maximumVoltage = 48.0f
    };


}

void PmsmControl::update(Types::pwmTicks_t currentPwmTick)
{
    ///
}


}