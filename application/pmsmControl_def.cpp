#include "pmsmControl/pmsmControl.h"

static uint32_t xMC_Ticks = 0;

inline mc_status PmsmControl::starting()
{
    starting_bemfzc();
    return mc_status::PMSM_OK;
}


inline mc_status PmsmControl::motorControlLoop()
{
    //Control Loop Code (read adcPwm scan data)
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

    ctx.cnt++;
}

inline mc_status PmsmControl::pwmControlLoop()
{
    ctx.t_nS += ctx.pwm_period_nS;
    ctx.t_uS += ctx.pwm_period_uS;
    adcpwmDriver.adcPwm_read(ctx.scanData);
    bemfZcd.update(ctx.t_nS, ctx.scanData[0], ctx.scanData[1], ctx.scanData[2], ctx.current_TRAP_Sector);
    return mc_status::PMSM_OK;
}


void MCADCPWM3P_adcPwm_postScanCallback(void)
{
    PmsmControl::pwmControlLoop();
}

void xMC_Tasker(void){
    PmsmControl::motorControlLoop();
    xMC_Ticks++;
}
