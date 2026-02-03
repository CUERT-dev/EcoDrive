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

#define SATURATE(v , min , max)(( v > max)?max: ((v < min)?0:v))
constexpr uint32_t CC1E_MASK = (TIM_CCER_CC1E);
constexpr uint32_t CC2E_MASK = (TIM_CCER_CC2E);
constexpr uint32_t CC3E_MASK = (TIM_CCER_CC3E);
constexpr uint32_t CC1NE_MASK = (TIM_CCER_CC1NE);
constexpr uint32_t CC2NE_MASK = (TIM_CCER_CC2NE);
constexpr uint32_t CC3NE_MASK = (TIM_CCER_CC3NE);

// Pre-calculate masks at compile time
constexpr uint32_t OC1M_MASK     = (0b111 << TIM_CCMR1_OC1M_Pos);
constexpr uint32_t OC1M_PWM      = (0b110 << TIM_CCMR1_OC1M_Pos);
constexpr uint32_t OC1M_INACTIVE = (0b100 << TIM_CCMR1_OC1M_Pos);

constexpr uint32_t OC2M_MASK     = (0b111 << TIM_CCMR1_OC2M_Pos);
constexpr uint32_t OC2M_PWM      = (0b110 << TIM_CCMR1_OC2M_Pos);
constexpr uint32_t OC2M_INACTIVE = (0b100 << TIM_CCMR1_OC2M_Pos);

constexpr uint32_t OC3M_MASK     = (0b111 << TIM_CCMR2_OC3M_Pos);
constexpr uint32_t OC3M_PWM      = (0b110 << TIM_CCMR2_OC3M_Pos);
constexpr uint32_t OC3M_INACTIVE = (0b100 << TIM_CCMR2_OC3M_Pos);


uint16_t Adc1_Sample_Single_Channel_Temporary(uint32_t channel);
void MCADCPWM3P_adc_calibrate();
//=======================================================================================================
// ADC Regular Scan Mode (for Application layer slow analog monitor (temperature, throttle , etc))
//=======================================================================================================
void MCADCPWM3P_adcReg_ScanModeInit();
void MCADCPWM3P_adcReg_dmaInit(const MCADCPWM3P_Config &cfg);
void MCADCPWM3P_adc_interruptInit(const MCADCPWM3P_Config &cfg);


#endif