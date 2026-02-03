#include "pmsmControl.h"

typedef enum : int8_t{
    PHASE_FLOAT = -1,      
    PHASE_LOW = 0,    
    PHASE_ACTIVE = 1    
} PhaseState_t;



constexpr PhaseState_t COMMUTATION_TABLE[8][3] = 
{// DUMMY FILLER
    {PHASE_LOW        , PHASE_LOW       , PHASE_LOW}, 

    {PHASE_ACTIVE     , PHASE_LOW       , PHASE_FLOAT},
    {PHASE_ACTIVE     , PHASE_FLOAT     , PHASE_LOW},
    {PHASE_FLOAT      , PHASE_ACTIVE    , PHASE_LOW},
    {PHASE_LOW        , PHASE_ACTIVE    , PHASE_FLOAT},
    {PHASE_LOW        , PHASE_FLOAT     , PHASE_ACTIVE},
    {PHASE_FLOAT      , PHASE_LOW       , PHASE_ACTIVE},

    {PHASE_ACTIVE     , PHASE_ACTIVE    , PHASE_ACTIVE}
};

// For trap sectors specifically (1-6)
#define TRAP_INCREMENT(sector)    INCREMENT_ROLL(sector, 1, 6)
#define TRAP_DECREMENT(sector)    DECREMENT_ROLL(sector, 1, 6)

void PmsmControl::hardwareInit()
{
    // Initialize PWM and Adc
    adcpwmDriver.init(__adcpwmConfig);
    ctx.pwm_period_nS = TICKS_TO_NANOS(1, __adcpwmConfig.pwmFreq_Hz);
    ctx.pwm_period_uS = TICKS_TO_MICROS(1, __adcpwmConfig.pwmFreq_Hz);  // Once
    // Load Motor Parameters
    motorParams = __motorParams;
    // Sensor Initialization code
    bemfZcd.init(__hallConfig);
    // Arm the Control Loop
}


#define ELAPSED_TIME_MS(ctx, start_time_us) ( ((ctx).t_nS / 1000000) - ((start_time_us) / 1000) )

mc_status PmsmControl::starting_bemfzc()
{
    switch (cts.startupPhase)
    {
    case StartupPhase::ALIGN_START:
    {
        //Alignment code
        sensorlessTrap_startup.align_startTime_uS = ctx.t_uS;
        ctx.current_TRAP_Sector = sensorlessTrap_startup.align.TRAP_Sector;
        ctx.voltage_q15 = (int16_t)((sensorlessTrap_startup.align.voltage_V / ctx.busVoltage_V) * 0x7FFF);
        cts.startupPhase = StartupPhase::ALIGN_RUNNING;
    }
    break;
    case StartupPhase::ALIGN_RUNNING:
    {
        if(ELAPSED_TIME_MS(ctx, sensorlessTrap_startup.align.start_time_uS) >= UNIT_TO_MILLI(sensorlessTrap_startup.align.duration_uS)){
            //Move to next phase
            sensorlessTrap_startup.ramp.start_time_uS = ctx.t_uS;
            cts.startupPhase = StartupPhase::OPENLOOP_RAMP_RUNNING;
        }
    }
    break;
    case StartupPhase::OPENLOOP_RAMP_RUNNING:
    {
        if (ELAPSED_TIME_MS(ctx, sensorlessTrap_startup.ramp.start_time_uS) < UNIT_TO_MILLI(sensorlessTrap_startup.ramp.ramp_duration_uS)) {
            //Voltage RAMP part
            float elapsed_time_s = (float)(ELAPSED_TIME_MS(ctx, sensorlessTrap_startup.ramp.start_time_uS)) / 1000.0f;
            float voltage_V = sensorlessTrap_startup.ramp.start_voltage_V + (elapsed_time_s * sensorlessTrap_startup.ramp.ramp_Voltage_VpS);
            ctx.voltage_q15 = (int16_t)((voltage_V / ctx.busVoltage_V) * 0x7FFF);

            //Frequency RAMP part
            float elecFrequency_Hz = sensorlessTrap_startup.ramp.start_ElecFrequency_Hz + (elapsed_time_s * sensorlessTrap_startup.ramp.ramp_ElecFrequency_HzpS);
            float elecPeriod_s = 1.0f / elecFrequency_Hz;
            sensorlessTrap_startup.ramp.stepDuration_uS = (uint32_t)(elecPeriod_s * 1000000.0f / 6.0f);
        }else{
            //Ramp complete , move to running
            sensorlessTrap_startup.ramp.rampComplete = true;
            cts.motorstate = MotorState::RUNNING;
            cts.startupPhase = StartupPhase::ALIGN_START; //Reset for next time
        }
    }
    break;
    default:
        break;
    }
}
