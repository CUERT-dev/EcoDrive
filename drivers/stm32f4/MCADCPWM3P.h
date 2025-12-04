#ifndef MCADCPWM3P_H
#define MCADCPWM3P_H

#include <stdint.h>

extern int16_t adc_to_uV;

#define ADC_TO_uV(adcval)((int32_t)(adcval) * adc_to_uV)
#define ADC_TO_mV(adcval)((int32_t)((adcval) * adc_to_uV)/1000)

void MCADCPWM3P_init(const MCADCPWM3P_Config &cfg);

uint16_t MCADCPWM3P_adcReg_read(uint8_t ch);
void MCADCPWM3P_adcPwm_read(int32_t scanData[6]);

void MCADCPWM3P_adcPwm_registerPostScanCallback(void (*callback)());
void MCADCPWM3P_pwm_write(int16_t dutyU_q15, int16_t dutyV_q15, int16_t dutyW_q15);


#endif