#pragma once
#include "driver/IMCADCPWM3P.h"
#include "driver/IBemfZc.h"
#include "core/math.hpp"

//===================================================
// CONFIG
//==================================================

struct pmsmParameter{
    uint8_t polePairs;
    uint32_t fluxLinkage_mWb;
    uint32_t statorResistance_mOhm;
    uint32_t statorInductance_uH;
    uint32_t inertia_uKgm2;
    uint32_t friction_uNmpS;
    uint32_t KV_mRpmPerVolt;
    uint32_t maximumCurrent_mA;
    uint32_t maximumVoltage_mV;
};

constexpr pmsmParameter __motorParams =  
{
    .polePairs = 7,
    .fluxLinkage_mWb = 15,
    .statorResistance_mOhm = 100,
    .statorInductance_uH = 1000,
    .inertia_uKgm2 = 1000,
    .friction_uNmpS = 1000,
    .KV_mRpmPerVolt = 120,
    .maximumCurrent_mA = 30,
    .maximumVoltage_mV = 48
};


constexpr MCADCPWM3P_Config __adcpwmConfig = 
{
    .pwmFreq_Hz = 10000,
    .deadtime_nS = 1300,
    .dutyMax_perc = 95,
    .dutyMin_perc = 5,
    .breakEnabled = false
};

constexpr Hall_Config __hallConfig = 
{
    .hysteresisVoltage_mV = 300,
    .commutationDelay_DegElec = 30,
    .minimumElecVelocity_DegpS = (1000*360)/60
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
    uint32_t busVoltage_mV = 12500;
    int32_t scanData[6];
    uint32_t cnt = 0;
    uint64_t t_nS = 0;
    uint32_t pwm_period_nS = 0;
    uint32_t t_uS = 0;
    uint16_t pwm_period_uS = 0;
    int16_t U_Duty_q15 = 0;
    int16_t V_Duty_q15 = 0;
    int16_t W_Duty_q15 = 0;
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


struct StartupData{
    // Phase 1: Alignment
    struct {
        uint32_t voltage_mV = UNIT_TO_MILLI(1);              // Alignment voltage (5V typical)
        uint32_t start_time_uS = 0;
        uint32_t duration_uS = UNIT_TO_MICRO(0.1);  // 100ms alignment
        uint8_t TRAP_Sector = 1;            // Which phase to align (0=U, 1=V, 2=W)
    } align;
    
    // Phase 2: Open-loop ramp
    struct {
        uint32_t start_time_uS = 0;
        uint32_t elecFrequency_Hz = 0;  
        uint32_t start_ElecFrequency_mHz = RPM_TO_HZ(UNIT_TO_MILLI(MECHFREQ_TO_ELECFREQ(100.0, __motorParams.polePairs)));
        uint32_t ramp_ElecFrequency_mHzpS = RPM_TO_HZ(UNIT_TO_MILLI(MECHFREQ_TO_ELECFREQ(1100.0, __motorParams.polePairs))) / 5.0; 
        
        uint32_t voltage_mV = 0;       // Start voltage
        uint32_t start_voltage_mV = UNIT_TO_MILLI(1); 
        uint32_t ramp_Voltage_mVpS = UNIT_TO_MILLI(2.0/5);     // Max voltage during open-loop

        uint32_t ramp_duration_uS = UNIT_TO_MICRO(5.0);
 
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
        int32_t zeroCrossAngleDetected_mDeg = 0;
        bool positionLocked = false;
        bool switchover_status = false;
    } switchover;
    
};


class PmsmControl{
    static inline MCADCPWM3P<1> adcpwmDriver;
    static inline pmsmParameter motorParams;
    // Position Sensors

    static inline BemfZc<1> bemfZcd;

    static inline controlLoopContext ctx;
    static inline ControlLoopState cts;
    static inline StartupData st_data;
    static void hardwareInit();
    //Operation modes
    static inline void starting();
    static inline void running();
    static inline void braking();
    static inline void calibrating();
    //starting modes
    static inline void starting_align();
    static inline void starting_bemfzc();
    static inline void starting_hall();
    static inline void starting_encoder();
 

  
    

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
    static inline void controlLoop();

};


