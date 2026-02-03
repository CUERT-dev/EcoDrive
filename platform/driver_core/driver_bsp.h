#ifndef DRIVER_GPIO_H
#define DRIVER_GPIO_H


#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_adc.h"

//########################################################################
// FIXED HARDWARE MAPPING FOR STM32F4xx Series MCU's
//########################################################################
// UART1 for Serial (B6, B7)
// SWD (A13, A14)
// USB1 (A11, A12)
// CAN1 (B8, B9) (if exists (doesnt exist for F401, F411))
// TIM1 for complementary PWM generation (A8, A9, A10, B13, B14, B15)
// TIM2 for Hall sensor interface / sensorless commutation timing (mocked triggers) (A15, B3, B10)
// TIM3 for encoder interface(B4, B5)

// ANALOG PINS(A0, A1, A2, A3, A4, A5, A6, A7, B0, B1, C0, C1, C2, C3, C4, C5)  
//Will Need 9 Pins Min For the Case of Triple Shunt + Phase Voltages + Bus Voltage + 1 Temperature + 1 Throttle

// USED ANALOG PINS (A0, A1, A2, A3, A4, A5, A6, A7, B0, B1)

// (BLACKPILL_F401 : FREE) : C13, C14, C15, B2. B12
//########################################################################
// FIXED HARDWARE MAPPING FOR STM32F4xx Series MCU's #########END#######
//########################################################################


//================================================
// MCADCPWM3P GPIO CONFIGURATION
//================================================
#define MCADCPWM3P_HIN_ACTIVE 1
#define MCADCPWM3P_LIN_ACTIVE 0

#define MCADCPWM3P_VSU_ADC_CHANNEL      LL_ADC_CHANNEL_1
#define MCADCPWM3P_VSV_ADC_CHANNEL      LL_ADC_CHANNEL_2
#define MCADCPWM3P_VSW_ADC_CHANNEL      LL_ADC_CHANNEL_3
#define MCADCPWM3P_CS                   CS_TRIPLE_SHUNT

#define MCADCPWM3P_VSBUS_ADC_CHANNEL    LL_ADC_CHANNEL_9   // PB1 - ADC1_IN9

//###############
// CURRENT SENSING CHANNELS (Configurable by CS type)
#if (MCADCPWM3P_CS == CS_TRIPLE_SHUNT)
    // Three shunt resistors - one per phase
    #define MCADCPWM3P_CSU_ADC_CHANNEL LL_ADC_CHANNEL_4   // PA3 - ADC1_IN4
    #define MCADCPWM3P_CSV_ADC_CHANNEL LL_ADC_CHANNEL_5   // PA4 - ADC1_IN5
    #define MCADCPWM3P_CSW_ADC_CHANNEL LL_ADC_CHANNEL_6   // PA5 - ADC1_IN6

#elif (MCADCPWM3P_CS == CS_DOUBLE_SHUNT)
    // Two shunt resistors - phases A & B only
    #define ADC_CHANNEL_PHASE_A_CURRENT    LL_ADC_CHANNEL_4   // PA3 - ADC1_IN4
    #define ADC_CHANNEL_PHASE_B_CURRENT    LL_ADC_CHANNEL_5   // PA4 - ADC1_IN5
    #define ADC_CHANNEL_PHASE_C_CURRENT    LL_ADC_CHANNEL_4   // Not used directly
    
#elif (MCADCPWM3P_CS == CS_SINGLE_SHUNT)
    // Single shunt resistor on DC bus
    #define ADC_CHANNEL_DC_BUS_CURRENT     LL_ADC_CHANNEL_4   // PA3 - ADC1_IN4
    
#elif (MCADCPWM3P_CS == CS_INLINE)
    // Inline current sensors (e.g., Hall effect, LEM)
    #define ADC_CHANNEL_PHASE_A_CURRENT    LL_ADC_CHANNEL_4   // PA3 - ADC1_IN4
    #define ADC_CHANNEL_PHASE_B_CURRENT    LL_ADC_CHANNEL_5   // PA4 - ADC1_IN5
    #define ADC_CHANNEL_PHASE_C_CURRENT    LL_ADC_CHANNEL_6   // PA5 - ADC1_IN6
    
#elif (MCADCPWM3P_CS == CS_NONE)
    // No current sensing 
#endif


// PWMSCAN conditioning macros  (Condition read voltage in mV to Physcial sensor values (current in mA and voltage in mV) )
#define U_CS_V_TO_A(c)(c*1)
#define V_CS_V_TO_A(c)(c*1)
#define W_CS_V_TO_A(c)(c*1)
#define U_VS_V_TO_V(v)(v * float((( 41500.0+2200.0)/2200.0)) )
#define V_VS_V_TO_V(v)(v * float((( 41200.0+2200.0)/2200.0)) )
#define W_VS_V_TO_V(v)(v * float((( 41500.0+2200.0)/2200.0)) )

#define BUS_CS_V_TO_A(v)()
#define BUS_VS_V_TO_V(v)(v * float((( 41500.0+2200.0)/2200.0)) )

//================================================
// HALL1 CONFIGURATION
//================================================
#define HALL1_A_ACTIVE 0
#define HALL1_B_ACTIVE 0
#define HALL1_C_ACTIVE 0

//================================================
// ENCODER1 CONFIGURATION
//================================================
#define ENCODER1_A_ACTIVE 0
#define ENCODER1_B_ACTIVE 0

//================================================
// UART1 CONFIGURATION
//================================================


//================================================
// USB CONFIGURATION
//================================================


//================================================
// CAN1 CONFIGURATION
//================================================


//================================================
//GPIO Initialization functions
//================================================
void MCADCPWM3P_adc_gpioInit();
void MCADCPWM3P_pwm_gpioInit();
void uart1_gpioInit();
void hall1_gpioInit();


//================================================
//GPIO UTILITY functions
//================================================
uint8_t hall1_gpioRead();


//================================================
// DEBUG/PROFILE GPIO CONFIGURATION
//================================================
#define DPIO_GPIO_PORT GPIOB
#define DPIO_GPIO_PIN LL_GPIO_PIN_2

#define DPIO_init() do{\
    LL_GPIO_SetPinMode(DPIO_GPIO_PORT, DPIO_GPIO_PIN, LL_GPIO_MODE_OUTPUT);\
    LL_GPIO_SetPinSpeed(DPIO_GPIO_PORT, DPIO_GPIO_PIN, LL_GPIO_SPEED_FREQ_MEDIUM);\
    LL_GPIO_SetPinOutputType(DPIO_GPIO_PORT, DPIO_GPIO_PIN, LL_GPIO_OUTPUT_PUSHPULL);\
    LL_GPIO_SetPinPull(DPIO_GPIO_PORT, DPIO_GPIO_PIN, LL_GPIO_PULL_NO);\
}while(0)

#define DPIO_set()(LL_GPIO_SetOutputPin(DPIO_GPIO_PORT, DPIO_GPIO_PIN))
#define DPIO_reset()(LL_GPIO_ResetOutputPin(DPIO_GPIO_PORT, DPIO_GPIO_PIN))
#define DPIO_NPULSE(n)do{\
    for(uint8_t i = 0; i < n; i++){\
        DPIO_set();\
        DPIO_reset();\
    }\
}while(0)

#endif


