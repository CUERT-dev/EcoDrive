#include "driver_bsp.h"
#include "driver_conf.h"



//================================================================================================================
// GPIO Initialization for MCADCPWM3P adc
//================================================================================================================
#define GPIO_SET_AF(port, pin, af) do { \
    if ((pin) <= LL_GPIO_PIN_7) { \
        LL_GPIO_SetAFPin_0_7(port, pin, af); \
    } else { \
        LL_GPIO_SetAFPin_8_15(port, pin, af); \
    } \
} while(0)



#define UART1_GPIOInit(port, pin)do{\
    LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_ALTERNATE);\
    LL_GPIO_SetPinSpeed(port, pin, LL_GPIO_SPEED_FREQ_HIGH);\
    LL_GPIO_SetPinOutputType(port, pin, LL_GPIO_OUTPUT_PUSHPULL);\
    LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_UP);\
    GPIO_SET_AF(port, pin, LL_GPIO_AF_7);\
}while(0)


#define MCADCPWM3P_ADC_GPIOInit(port, pin)do{\
    LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_ANALOG);\
    LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_NO);\
}while(0)


#define MCADCPWM3P_PWM_GPIOInit(port, pin, active)do{\
    LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_ALTERNATE);\
    LL_GPIO_SetPinSpeed(port, pin, LL_GPIO_SPEED_FREQ_LOW);\
    LL_GPIO_SetPinOutputType(port, pin, LL_GPIO_OUTPUT_PUSHPULL);\
    LL_GPIO_SetPinPull(port, pin, active?LL_GPIO_PULL_DOWN : LL_GPIO_PULL_UP);\
    GPIO_SET_AF(port, pin, LL_GPIO_AF_1);\
}while(0)

#define HALL_GPIOInit(port, pin)do{\
    LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_INPUT);\
    LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_NO);\
    LL_GPIO_SetPinSpeed(port, pin, LL_GPIO_SPEED_FREQ_LOW);\
}while(0)

void MCADCPWM3P_adc_gpioInit()
{

    MCADCPWM3P_ADC_GPIOInit(GPIOA, LL_GPIO_PIN_0);
    MCADCPWM3P_ADC_GPIOInit(GPIOA, LL_GPIO_PIN_1);
    MCADCPWM3P_ADC_GPIOInit(GPIOA, LL_GPIO_PIN_2);
    // Optional: External VREF if used
    #ifdef ADC_EXTERNAL_VREF

    #endif
}

//================================================================================================================
// GPIO Initialization for MCADCPWM3P pwm
//================================================================================================================

void MCADCPWM3P_pwm_gpioInit()
{
    // HIGH SIDE GPIO CONFIG (PA8, PA9, PA10)
    MCADCPWM3P_PWM_GPIOInit(GPIOA, LL_GPIO_PIN_8, MCADCPWM3P_HIN_ACTIVE);
    MCADCPWM3P_PWM_GPIOInit(GPIOA, LL_GPIO_PIN_9, MCADCPWM3P_HIN_ACTIVE);
    MCADCPWM3P_PWM_GPIOInit(GPIOA, LL_GPIO_PIN_10, MCADCPWM3P_HIN_ACTIVE);
    MCADCPWM3P_PWM_GPIOInit(GPIOB, LL_GPIO_PIN_13, MCADCPWM3P_LIN_ACTIVE);
    MCADCPWM3P_PWM_GPIOInit(GPIOB, LL_GPIO_PIN_14, MCADCPWM3P_LIN_ACTIVE);
    MCADCPWM3P_PWM_GPIOInit(GPIOB, LL_GPIO_PIN_15, MCADCPWM3P_LIN_ACTIVE);
}

//================================================================================================================
// GPIO Initialization for Uart1
//================================================================================================================

void uart1_gpioInit()
{
    // Configure PB6 (TX)
    UART1_GPIOInit(GPIOB, LL_GPIO_PIN_6);
    // Configure PB7 (RX)
    UART1_GPIOInit(GPIOB, LL_GPIO_PIN_7);

}


//================================================================================================================
// GPIO Initialization for hall sensor 1
//================================================================================================================

void hall1_gpioInit(void)
{
    // PWM Scan Channels (Time-Critical Measurements)

}


uint8_t hall1_gpioRead()
{
    return 0;
}