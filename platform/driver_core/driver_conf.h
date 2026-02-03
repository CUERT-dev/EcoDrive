#ifndef DRV_CONF_H
#define DRV_CONF_H
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_rcc.h"


/*############################################################
//DOCUMENTATION

DEPENEDENCIES:
MCADCPWM3P <<---- pwm3p1

COMMON:
driver_conf.h (Has Driver configurations for what drivers to use , Buffers size , and similar for tuning driver layer)
driver_bsp.h ("Board Support Package"  Has Gpio Initialization functions , needed GPIO macros, Adc Channel mapping)
common.h
*/


//================================================
// GENERAL DRIVER LAYER CONFIGURATION
//================================================
#define DRIVER_USE_UART_I1
#define DRIVER_USE_PWM3P_I1
#define DRIVER_USE_MCADCPWM3P

#define UART_I1_DMA_RECEIVE_COMPLETE_CALLBACK_ENABLED 0



constexpr uint32_t UART1_TX_DRV_BUFFER_SIZE = 256;
constexpr uint32_t UART1_RX_DRV_BUFFER_SIZE = 256;


constexpr uint8_t MCADCPWM3P_REGSCAN_CHANNEL_NUM = 3;
//Motor control tasking frequency
#define XMC_TICKFREQ_HZ 2000
//================================================
// DMA CONFIGURATION
//================================================


//================================================
// NVIC CONFIGURATION
//================================================



//#define ADC_EXTERNAL_VREF
#define ADC_RESOLUTION_BITS 12
#define MCADCPWM3P_USE_INTTEMP
#define VREFEXT_CAL_VOLTAGE 2.495f


#endif//driver_conf.h