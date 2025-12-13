#include "MCADCPWM3P_common.h"

#define ST_TASK(task_name, ticker, task, divisor)do{\
    static uint16_t counter_##task_name = 0;\
    if(++counter_##task_name >= divisor){\
        counter_##task_name = 0;\
        task;\
    }\
}\
while(0);

__attribute__((weak)) void xMC_Tasker(void)
{
    // Empty - user can override this
}

__attribute__((weak)) void MCADCPWM3P_adcPwm_postScanCallback(void)
{
    // Empty - user can override this
}

extern "C" {
    void ADC_IRQHandler(void)
    {
    if (LL_ADC_IsActiveFlag_JEOS(ADC1)) {
        LL_ADC_ClearFlag_JEOS(ADC1);
        MCADCPWM3P_adcPwm_postScanCallback();
    }
    }

    void TIM1_UP_TIM10_IRQHandler(void)
    {
        if (LL_TIM_IsActiveFlag_UPDATE(TIM1)) {
            LL_TIM_ClearFlag_UPDATE(TIM1);
        }
    }

    void TIM1_TRG_COM_TIM11_IRQHandler(void)
    {
        //=============TIMER 11 UPDATE INTERRUPT==============// USED for HAL_Ticker and MC_Scheduler
        if (LL_TIM_IsActiveFlag_UPDATE(TIM11)){
            LL_TIM_ClearFlag_UPDATE(TIM11);
            //Here We share Timer11 for HAL_ticking and motor Control Scheduling
            static uint8_t trg_cnt = 0;
            xMC_Tasker();
            ST_TASK(HAL_TICKER, trg_cnt, HAL_IncTick(), XMC_TICKFREQ_HZ/1000);
        }
    }
}


