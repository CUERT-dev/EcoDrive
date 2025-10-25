#pragma once


#include <stdint.h>

namespace EcoDrive{
namespace Types{

enum class Polarity : bool{
    ActiveLow = false,
    ActiveHigh = true
};


enum class CurrentSensorType: uint8_t{
    none = 0,
    singleLowShunt,
    doubleLowShunt,
    tripleLowShunt,
    directInline
};

enum class VoltageSensorType: uint8_t{
    none = 0,
    resistiveDivider = 1
};

enum class BemfZcType: uint8_t{
    triplePhase = 0,
    floatingPhase_dcBus,
    floatingPhase_virtualNeutral
};

enum class HallSensorErrorType: uint8_t{
    noError = 0,
    invalidState,
    notConnected,
    badSequence
};

enum class HallSensorType: uint8_t{
    none = 0,
    unipolar
};

enum class InternalAdcReferenceType: uint8_t{
    none = 0,
    bandgap1V2,
    bandgap2V5,
    external
};

typedef uint32_t pwmTicks_t;

enum class PWMAlignmentType: uint8_t{
    centerAligned = 0,
    edgeAligned
};

enum class PWMModulationType: uint8_t{
    PWM = 0,
    SPWM,
    SVM
};

};
};
