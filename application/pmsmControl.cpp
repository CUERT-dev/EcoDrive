#include "pmsmControl.hpp"

#define ST_TASK(task_name, ticker, task, divisor)do{\
    static uint16_t counter_##task_name = 0;\
    if(++counter_##task_name > divisor){\
        counter_##task_name = 0;\
        task;\
    }\
}\
while(0);

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

static inline int32_t TRAP_toElecAngle_mDeg(uint8_t TRAP_sector){
    switch (TRAP_sector)
    {
    case 1:
        return 0;
        break;
    case 2:
        return UNIT_TO_MILLI(60);
        break;
    case 3:
        return UNIT_TO_MILLI(120);
        break;
    case 4:
        return UNIT_TO_MILLI(180);
        break;
    case 5:
        return UNIT_TO_MILLI(240);
        break;
    case 6:
        return UNIT_TO_MILLI(300);
        break;
    
    default:
        return 0xFFFFFFFF;
        break;
    }
}

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
    adcpwmDriver.adcPwm_registerPostScanCallback(controlLoop);
}


void PmsmControl::controlLoop()
{
    DPIO_set();
    ctx.t_nS += ctx.pwm_period_nS;
    ctx.t_uS += ctx.pwm_period_uS;
    //Control Loop Code (read adcPwm scan data)
    adcpwmDriver.adcPwm_read(ctx.scanData);
    switch (cts.motorstate)
    {
    case MotorState::STARTING:  
        starting();
        break;
    case MotorState::RUNNING:
        break;
    default:
        break;
    }
    adcpwmDriver.pwm_write(ctx.U_Duty_q15, ctx.V_Duty_q15, ctx.W_Duty_q15);
    ctx.cnt++;
    DPIO_reset();
}


static inline void TRAP_write(controlLoopContext &ctx, uint8_t TRAP_sector, int16_t magnitude_q15)
{
    switch (TRAP_sector)
    {
    case 1:
        ctx.U_Duty_q15 = magnitude_q15;
        ctx.V_Duty_q15 = 0;
        ctx.W_Duty_q15 = -1;
        break;
    case 2:
        ctx.U_Duty_q15 = magnitude_q15;
        ctx.V_Duty_q15 = -1;
        ctx.W_Duty_q15 = 0;
        break;
    case 3:
        ctx.U_Duty_q15 = -1;
        ctx.V_Duty_q15 = magnitude_q15;
        ctx.W_Duty_q15 = 0;
        break;
    case 4:
        ctx.U_Duty_q15 = 0;
        ctx.V_Duty_q15 = magnitude_q15;
        ctx.W_Duty_q15 = -1;
        break;
    case 5:
        ctx.U_Duty_q15 = 0;
        ctx.V_Duty_q15 = -1;
        ctx.W_Duty_q15 = magnitude_q15;
        break;
    case 6:
        ctx.U_Duty_q15 = -1;
        ctx.V_Duty_q15 = 0;
        ctx.W_Duty_q15 = magnitude_q15;
        break;
    default:
        break;
    }

    ctx.current_TRAP_Sector = TRAP_sector;
}


inline void PmsmControl::starting()
{
    starting_bemfzc();
}



inline void PmsmControl::starting_bemfzc()
{
    switch (cts.startupPhase)
    {
    case StartupPhase::ALIGN_START:{
        ctx.current_TRAP_Sector = st_data.align.TRAP_Sector;
        st_data.align.start_time_uS = ctx.t_uS; //record start time
        int16_t duty_q15 = int32_t(st_data.align.voltage_mV<< 15)/ctx.busVoltage_mV;
        TRAP_write(ctx, ctx.current_TRAP_Sector, duty_q15); 
        bemfZcd.reset(TRAP_toElecAngle_mDeg(ctx.current_TRAP_Sector)); //Reset backemf zero cross position sensor position
        cts.startupPhase = StartupPhase::ALIGN_RUNNING; // move to next state , waiting
        break;
    }
    case StartupPhase::ALIGN_RUNNING:{
        uint32_t delta_t_uS = ctx.t_uS - st_data.align.start_time_uS;
        if (delta_t_uS > st_data.align.duration_uS)
        {
            st_data.ramp.start_time_uS = ctx.t_uS;
            st_data.ramp.lastStepTime_uS = ctx.t_uS;
            cts.startupPhase = StartupPhase::OPENLOOP_RAMP_RUNNING;
        }
        break;
    }
    case StartupPhase::OPENLOOP_RAMP_RUNNING:{
        uint32_t delta_t_uS = ctx.t_uS - st_data.ramp.start_time_uS;
        uint32_t delta_t2_uS = ctx.t_uS - st_data.ramp.lastStepTime_uS;
        if (delta_t_uS < st_data.ramp.ramp_duration_uS)
        {
            st_data.ramp.voltage_mV = st_data.ramp.start_voltage_mV + MICRO_TO_UNIT( int64_t(st_data.ramp.ramp_Voltage_mVpS * delta_t_uS) ); //Increment voltage and ramp it as long we havent crossed ramp duration
            st_data.ramp.elecFrequency_Hz = MILLI_TO_UNIT(st_data.ramp.start_ElecFrequency_mHz) + NANO_TO_UNIT(int64_t(st_data.ramp.ramp_ElecFrequency_mHzpS) * delta_t_uS); //Increment Frequency as long as we havent crossed ramp duration
        }
        if(delta_t2_uS >= st_data.ramp.stepDuration_uS)
        {
            st_data.ramp.lastStepTime_uS = ctx.t_uS;
            st_data.ramp.stepDuration_uS = UNIT_TO_MICRO(1)/((st_data.ramp.elecFrequency_Hz)*6);
            if((ctx.direction==Direction::FORWARD)){
               TRAP_INCREMENT(ctx.current_TRAP_Sector);
            }else {
               TRAP_DECREMENT(ctx.current_TRAP_Sector);
            }
        }
        int16_t duty_q15 = int32_t(st_data.ramp.voltage_mV << 15)/ctx.busVoltage_mV;
        TRAP_write(ctx, ctx.current_TRAP_Sector, duty_q15);
        bemfZcd.update(ctx.t_uS, ctx.scanData[0], ctx.scanData[1], ctx.scanData[2], ctx.current_TRAP_Sector);
        break;
    }
    default:
        break;
    }
}
