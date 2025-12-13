#ifndef MCADCPWM3P_H
#define MCADCPWM3P_H

#include <stdint.h>
#include "driver_common.h"

extern int16_t adc_to_uV;

inline int32_t MCADCPWM3P_ADC_TO_uV(uint16_t adcval){return ((int32_t)(adcval) * adc_to_uV);}
inline int32_t MCADCPWM3P_ADC_TO_mV(uint16_t adcval){return (((int32_t)(adcval) * adc_to_uV)/1000);}

void MCADCPWM3P_init(const MCADCPWM3P_Config &cfg);

void MCADCPWM3P_adcReg_startConv();
const int32_t* MCADCPWM3P_adcReg_read(void);
bool MCADCPWM3P_adcReg_isReady(void);

void MCADCPWM3P_adcPwm_read(int32_t scanData[6]);
void MCADCPWM3P_adcPwm_setScanMode(MCADCPWM3P_PwmScanMode mode);

void MCADCPWM3P_pwmFLOAT_write();
void MCADCPWM3P_pwmTRAP_write(int8_t sector, uint16_t duty_q15);
void MCADCPWM3P_pwmSVM_write(int16_t dutyU_q15, int16_t dutyV_q15, int16_t dutyW_q15);

__attribute__((weak)) void xMC_Tasker(void);
__attribute__((weak)) void MCADCPWM3P_adcPwm_postScanCallback(void);

#endif