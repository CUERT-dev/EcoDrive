#ifndef MCADCPWM3P_COMMON_H
#define MCADCPWM3P_COMMON_H
#include "../MCADCPWM3P.h"

extern float adcVoltageRef_volt;
extern float internalRefVoltage_volt;
extern const float externalRefVoltage_volt;
extern void(*adcPwm_postScanCallback)();

uint16_t Adc1_Sample_Single_Channel_Temporary(uint32_t channel);
void MCADCPWM3P_adc_calibrate();
//=======================================================================================================
// ADC Regular Scan Mode (for Application layer slow analog monitor (temperature, throttle , etc))
//=======================================================================================================
void MCADCPWM3P_adcReg_ScanModeInit();
void MCADCPWM3P_adcReg_dmaInit(const MCADCPWM3P_Config &cfg);
void MCADCPWM3P_adc_interruptInit(const MCADCPWM3P_Config &cfg);

uint16_t MCADCPWM3P_adcReg_read(uint16_t ch);

inline void MCADCPWM3P_adcReg_startConv()
{
    LL_ADC_REG_StartConversionSWStart(ADC1);
}

#endif