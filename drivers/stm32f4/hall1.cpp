#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "hall1.h"

#define CLK_FREQ 1000000  // 1 MHz = 1us resolution
#define CLK_TO_US(clk)(clk * 1000000/CLK_FREQ)
#define US_TO_CLK(us)(us * (CLK_FREQ / 1000000))

void(*commutation_callback)();
volatile uint8_t hall1_value = 000;
volatile uint16_t hall1_period_uS = 0;

void hall1_tim2Init(void)
{
    // Enable Timer 2 clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    
    // ===== TIME BASE CONFIGURATION =====
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    TIM_InitStruct.Prescaler =  __LL_TIM_CALC_PSC(HAL_RCC_GetPCLK1Freq(), CLK_FREQ); // 84MHz/84 = 1MHz (1us resolution)
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 0xFFFF; // Max 16-bit value (65.535ms)
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM2, &TIM_InitStruct);

    // ===== HALL SENSOR CONFIGURATION =====
    LL_TIM_HALLSENSOR_InitTypeDef HALL_InitStruct = {0};
    HALL_InitStruct.IC1Polarity = LL_TIM_IC_POLARITY_BOTHEDGE; // Capture any Hall change
    HALL_InitStruct.IC1Prescaler = LL_TIM_ICPSC_DIV1;          // Capture every edge
    HALL_InitStruct.IC1Filter = 0x7;                           // 8 samples filtering
    HALL_InitStruct.CommutationDelay = 0;                      // Will set later
    LL_TIM_HALLSENSOR_Init(TIM2, &HALL_InitStruct);
    
    // ===== START TIMER =====
    LL_TIM_EnableCounter(TIM2);
}

void hall1_interruptInit()
{
    // ===== INTERRUPT CONFIGURATION =====
    // Enable capture/compare interrupt for Channel 1 (Hall changes)
    LL_TIM_EnableIT_UPDATE(TIM2);
    // Enable update interrupt for Channel 2 (commutation delay)
    LL_TIM_EnableIT_CC2(TIM2);
    
    // NVIC Configuration
    NVIC_SetPriority(TIM2_IRQn, 2); // Lower priority than PWM timer
    NVIC_EnableIRQ(TIM2_IRQn);
}

void hall1_init()
{   
    hall1_tim2Init();
    hall1_interruptInit();

    #ifdef HALL1_HARDWARE
    hall1_gpioInit();
    #endif

    //Arm Hall1 value
    hall1_value = hall1_gpioRead();
}

void hall1_triggerSWChange(void)
{
    //Trigger a hall sensor change effect in software (this would be exactlly equivelant to pin state change on hall sensors)
    LL_TIM_GenerateEvent_CC1(TIM2);
}

void hall1_setCOMDelay(uint32_t COM_delay_uS)
{
    LL_TIM_OC_SetCompareCH2(TIM2, US_TO_CLK(COM_delay_uS));
}

// Set your commutation callback function
void hall1_setCOMEVENTCallback(void (*callback)(void))
{
    commutation_callback = callback;
}



extern "C"{
    // Timer 2 Interrupt Handler - THIS IS YOUR COMMUTATION EVENT!
    void TIM2_IRQHandler(void)
    {

        if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
            LL_TIM_ClearFlag_UPDATE(TIM2);
            // HALL change happened , capture the period and update the hall_value using hall1_gpioRead();
            hall1_period_uS = CLK_TO_US(LL_TIM_IC_GetCaptureCH1(TIM2));
            
            #ifdef HALL1_HARDWARE
            hall1_value = hall1_gpioRead();
            #endif
        }

        if (LL_TIM_IsActiveFlag_CC2(TIM2)) {
            LL_TIM_ClearFlag_CC2(TIM2);
            // PERFORM COMMUTATION HERE
            if (commutation_callback != NULL) {
                commutation_callback();
            }
        }
    }
}
