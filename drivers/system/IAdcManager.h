#pragma once
#include "core/motorControl/types.h"

constexpr uint16_t ADC_SCAN_DMABUFFER_LENGTH = 16;
constexpr uint16_t  THROTTLE_SAMPLE_RATE_HZ  = 100;
constexpr uint16_t TEMP_SAMPLE_RATE_HZ = 10;


namespace EcoDrive{

class AdcManager{
    enum class AdcChannel {
        PhaseCurrentA,
        PhaseCurrentB,
        PhaseCurrentC,
        PhaseVoltageA,
        PhaseVoltageB,
        PhaseVoltageC,
        BusVoltage,
        ThrottleVoltage,
        TemperatureSensor1,
        TemperatureSensor2,
        CustomSensor1,
        CustomSensor2
    };


    struct AdcData{
        uint16_t phaseCurrentA_raw;
        uint16_t phaseCurrentB_raw;
        uint16_t phaseCurrentC_raw;
        uint16_t phaseVoltageA_raw;
        uint16_t phaseVoltageB_raw;
        uint16_t phaseVoltageC_raw;
        uint16_t busVoltage_raw;
        uint16_t throttleVoltage_raw;
        uint16_t temperatureSensor1_raw;
        uint16_t temperatureSensor2_raw;
        uint16_t customSensor1_raw;
        uint16_t customSensor2_raw;
        uint16_t adcVoltageRef_raw;
    };


    uint16_t DMA_BUFFER[ADC_SCAN_DMABUFFER_LENGTH];
    void* postScanCallback = nullptr;
    AdcData adcData;
    void calibrate();

    public:
    struct Config{    
        Types::VoltageSensorType voltageSensorType;
        Types::CurrentSensorType currentSensorType;
        uint32_t samplePwmEdgeDelay_ns;
        Types::InternalAdcReferenceType adcVoltageRefType;
        float adcVoltageRef_volt;
    };

    
    void init(const Config cfg);
    void update(Types::pwmTicks_t currentPwmTick);
    void registerPostScanCallback(void (*callback)());
    const AdcData& getAdcData() const;
    ~AdcManager() = default;

};

};
