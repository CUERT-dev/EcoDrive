#pragma once
#include "core/math.hpp"
#include "driver/IMCADCPWM3P.h"
#include "driver/IBemfZc.h"
//===================================================
// CONFIG
//==================================================


#define ST_TASK(task_name, ticker, task, divisor)do{\
    static uint16_t counter_##task_name = 0;\
    if(++counter_##task_name >= divisor){\
        counter_##task_name = 0;\
        task;\
    }\
}\
while(0);

struct pmsmParameter{
    uint8_t polePairs;
    float fluxLinkage_Wb;
    float statorResistance_Ohm;
    float statorInductance_H;
    float inertia_Kgm2;
    float friction_NmpS;
    float KV_RpmPerVolt;
    float maximumCurrent_A;
    float maximumVoltage_V;
};

constexpr pmsmParameter __motorParams =  
{
    .polePairs = 7,
    .fluxLinkage_Wb = 15,
    .statorResistance_Ohm = 100,
    .statorInductance_H = 1000,
    .inertia_Kgm2 = 1000,
    .friction_NmpS = 1000,
    .KV_RpmPerVolt = 120,
    .maximumCurrent_A = 30,
    .maximumVoltage_V = 48
};

constexpr MCADCPWM3P_Config __adcpwmConfig = 
{
    .pwmFreq_Hz = 20000,
    .deadtime_nS = 1500,
    .dutyMax_perc = 15,
    .dutyMin_perc = 5,
    .breakEnabled = false
};

constexpr Hall_Config __hallConfig = 
{
    .hysteresisVoltage_V = 0.3,
    .commutationDelay_DegElec = 30,
    .minimumElecVelocity_DegpS = (360*5)/60
};

//===================================================
// Definitions
//==================================================
enum class MotorState : uint8_t {
    IDLE = 0,           // No power, PWM disabled
    STARTING,           // Startup sequence (open-loop ramp, etc.)
    RUNNING,            // Normal operation
    STOPPING,           // Controlled deceleration
    COASTING,           // Freewheel
    BRAKING,            // Active braking
    FAULT,              // Error state
    CALIBRATING         // Sensor calibration
};

// Level 2: Control Strategy (how we control it)
enum class ControlStrategy : uint8_t {
    OPENCOM = 0,            // V/f control
    TRAP,
    FOC
};

enum class StartupPhase : uint8_t {
    ALIGN_START = 0,
    ALIGN_RUNNING,
    OPENLOOP_RAMP_RUNNING
};

enum class PositionSensor : uint8_t {
    HALL = 0,
    BEMFZC,
    ENCODER,
    OBSERVER
};


// Level 5: Running sub-mode (only valid when MotorState = RUNNING)
enum class RunMode : uint8_t {
    SPEED_CONTROL = 0,  // Speed regulation
    TORQUE_CONTROL,     // Torque regulation
    POSITION_CONTROL,   // Position servo
    POWER_LIMITING,     // Power/current limiting    NOT_STARTING = 0,
    FIELD_WEAKENING     // High-speed operation
};

enum class Direction : uint8_t {
    FORWARD,
    REVERSE
};


struct controlLoopContext{
    uint32_t busVoltage_V = 12.5;
    int32_t scanData[6];
    uint32_t cnt = 0;
    uint64_t t_nS = 0;
    uint32_t pwm_period_nS = 0;
    uint32_t t_uS = 0;
    uint16_t pwm_period_uS = 0;
    
    int16_t voltage_q15 = 0;
    uint8_t current_TRAP_Sector = 0; 
    

    
    Direction direction = Direction::FORWARD; 
};

struct ControlLoopState{
    MotorState motorstate = MotorState::STARTING;
    ControlStrategy controlStrategy = ControlStrategy::TRAP;
    RunMode runMode = RunMode::SPEED_CONTROL;
    StartupPhase startupPhase = StartupPhase::ALIGN_START;
    PositionSensor positionSensor = PositionSensor::BEMFZC;
};

struct SensorlessTrap_StartupData{
    // Phase 1: Alignment
    struct {
        float voltage_V = 1;              // Alignment voltage (5V typical)
        uint32_t start_time_uS = 0;
        uint32_t duration_uS = UNIT_TO_MICRO(0.1);  // 100ms alignment
        uint8_t TRAP_Sector = 1;            // Which phase to align (0=U, 1=V, 2=W)
    } align;
    
    // Phase 2: Open-loop ramp
    struct {
        uint32_t start_time_uS = 0;
        float elecFrequency_Hz = 0;  
        float start_ElecFrequency_Hz = RPM_TO_HZ(MECHFREQ_TO_ELECFREQ(10.0, __motorParams.polePairs));
        float ramp_ElecFrequency_HzpS = RPM_TO_HZ(MECHFREQ_TO_ELECFREQ(50.0, __motorParams.polePairs)) / 20.0; 
        
        float start_voltage_V = 1; 
        float ramp_Voltage_VpS = 3.0/20;     // Max voltage during open-loop

        uint32_t ramp_duration_uS = UNIT_TO_MICRO(20.0);
 
        uint32_t lastStepTime_uS = 0;      // Last commutation time

        uint32_t stepDuration_uS = 0;
        
        bool rampComplete = 0;
    } ramp;
        
    // Phase 3: Switchover detection
    struct {
        uint32_t stableCycles = 0;           // How many cycles stable
        uint32_t requiredStableCycles = 10;  // Need 10 stable cycles
        uint32_t maxTimeout_uS = UNIT_TO_MICRO(150);
        uint32_t lastZeroCrossTime_uS = 0;
        int32_t zeroCrossAngleDetected_Deg = 0;
        bool positionLocked = false;
        bool switchover_status = false;
    } switchover;
    
};

enum class mc_status: uint8_t {
    PMSM_OK = 0,
    PMSM_ERROR_INVALID_STATE,
    PMSM_ERROR_OVERCURRENT,
    PMSM_ERROR_OVERVOLTAGE,
    PMSM_ERROR_UNDERVOLTAGE,
    PMSM_ERROR_OVERTEMPERATURE
};


class PmsmControl{
    static inline MCADCPWM3P<1> adcpwmDriver;
    static inline pmsmParameter motorParams;
    // Position Sensors
    static inline BemfZc<1> bemfZcd;
    static inline controlLoopContext ctx;
    static inline ControlLoopState cts;
    static inline SensorlessTrap_StartupData sensorlessTrap_startup;
    static void hardwareInit();
    
    //Operation modes
    static inline mc_status starting();
    static inline mc_status running();
    static inline mc_status braking();
    static inline mc_status calibrating();

    static mc_status starting_bemfzc();
public:

    static inline void init()
    {
        hardwareInit();
    }

    static inline void update(uint32_t t_ms)
    {

    }

    static inline void setTargetSpeed()
    {

    }

    static inline float getCurrentSpeed_RPM()
    {
        return float();
    }

    static inline void setTargetCurrent()
    {

    }

    ~PmsmControl() = default;
      //Control Superloop
    static inline mc_status pwmControlLoop();
    static inline mc_status motorControlLoop();

};


