#ifndef MCADCPWM3P_COMMON_H
#define MCADCPWM3P_COMMON_H
#include "../MCADCPWM3P.h"
#include "stm32f401xc.h"

#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"

#include "../driver_conf.h"
#include "../driver_bsp.h"

extern float adcVoltageRef_volt;
extern float internalRefVoltage_volt;
extern const float externalRefVoltage_volt;

uint16_t Adc1_Sample_Single_Channel_Temporary(uint32_t channel);
void MCADCPWM3P_adc_calibrate();
//=======================================================================================================
// ADC Regular Scan Mode (for Application layer slow analog monitor (temperature, throttle , etc))
//=======================================================================================================
void MCADCPWM3P_adcReg_ScanModeInit();
void MCADCPWM3P_adcReg_dmaInit(const MCADCPWM3P_Config &cfg);
void MCADCPWM3P_adc_interruptInit(const MCADCPWM3P_Config &cfg);


#endif