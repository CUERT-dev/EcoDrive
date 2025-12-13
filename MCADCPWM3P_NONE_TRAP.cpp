#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f401xc.h"

#include "../driver_conf.h"
#include "../driver_bsp.h"
#include "../MCADCPWM3P.h"
#include "MCADCPWM3P_common.h"

//======================================================
//DRIVER LAYER GLOBALS
//======================================================
MCADCPWM3P_Instance MCADCPWM3P_I1;
MCADCPWM3P_Config MCADCPWM3P_CFG1;
void(*adcPwm_postScanCallback)();

static void MCADCPWM3P_tim1Init(const MCADCPWM3P_Config &cfg){
    //ENABLE Peripheral CLOCK
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    //===================================================== 
    // BASIC TIMER CONFIGURATION
    //=====================================================
    LL_TIM_InitTypeDef tim1Init =
    {
        .Prescaler = 0,
        .CounterMode = LL_TIM_COUNTERMODE_CENTER_UP,
        .Autoreload = __LL_TIM_CALC_ARR(HAL_RCC_GetPCLK2Freq(), 1, cfg.pwmFreq_Hz),
        .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 0
    };
    LL_TIM_EnableARRPreload(TIM1);
    LL_TIM_SetUpdateSource(TIM1, LL_TIM_UPDATESOURCE_REGULAR);
    LL_TIM_Init(TIM1, &tim1Init);
    //===================================================== 
    // ADVANCED TIMER CONFIGURATION (BREAK AND DEADTIME)
    //=====================================================
    LL_TIM_BDTR_InitTypeDef tim1BDTRInit = {
        .OSSRState = LL_TIM_OSSR_ENABLE,
        .OSSIState = LL_TIM_OSSI_DISABLE,
        .LockLevel = LL_TIM_LOCKLEVEL_OFF,
        .DeadTime = static_cast<uint8_t>(__LL_TIM_CALC_DEADTIME(HAL_RCC_GetPCLK2Freq(), LL_TIM_GetClockDivision(TIM1), cfg.deadtime_nS)),
        .BreakState = LL_TIM_BREAK_DISABLE,
        .BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH,
        .AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE
    };
    LL_TIM_BDTR_Init(TIM1, &tim1BDTRInit);
    //===================================================== 
    // IDENTICAL OUTPUT CHANNEL CONFIGURATION ON CHANNEL 1,2,3
    //=====================================================
    LL_TIM_OC_InitTypeDef ch1ch2ch3_tim1_OCInit = 
    {
        .OCMode = LL_TIM_OCMODE_INACTIVE,
        .OCState = LL_TIM_OCSTATE_ENABLE,
        .OCNState = LL_TIM_OCSTATE_DISABLE,
        .CompareValue = 0,
        .OCPolarity = MCADCPWM3P_HIN_ACTIVE ? LL_TIM_OCPOLARITY_HIGH : LL_TIM_OCPOLARITY_LOW,
        .OCNPolarity = MCADCPWM3P_LIN_ACTIVE ? LL_TIM_OCPOLARITY_HIGH : LL_TIM_OCPOLARITY_LOW,
        .OCIdleState = MCADCPWM3P_HIN_ACTIVE ? LL_TIM_OCIDLESTATE_LOW: LL_TIM_OCIDLESTATE_HIGH,
        .OCNIdleState = MCADCPWM3P_LIN_ACTIVE ? LL_TIM_OCIDLESTATE_LOW : LL_TIM_OCIDLESTATE_HIGH
    };
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &ch1ch2ch3_tim1_OCInit);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &ch1ch2ch3_tim1_OCInit);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &ch1ch2ch3_tim1_OCInit);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnablePreload(TIM1);
   
   
    //GET TIMER MAX MAX VALUE TO BE USED FOR DUTY CALCULATION
    MCADCPWM3P_I1.timerMax_q15 = LL_TIM_GetAutoReload(TIM1);
    // Finally enable the timer (Configuration done)
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
}

//======================================================
//Interrupt Triggered at Update event for TIM1
//======================================================
static void MCADCPWM3P_tim1InterruptInit(const MCADCPWM3P_Config &cfg)
{
    // Enable update interrupt (triggers at peak AND valley of center-aligned PWM)
    LL_TIM_EnableIT_UPDATE(TIM1);

    // Configure NVIC for TIM1 update interrupt
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 3);  // Set appropriate priority
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

//===================================================================
//Configuration for ADC pwm rate scanned data in trapezoidal mode
//===================================================================
static void MCADCPWM3P_adcPwm_TrapScanMode()
{
    //## Configure use TIM1 CH4 for adc triggering (we exlusively use CH4 for adc triggering)
    //==================================================================
    // USE CHANNEL 4 Exlusively FOR Injected Adc channel (adcPWM Scan)
    //==================================================================
    LL_TIM_OC_InitTypeDef ch4_tim1_OCInit = 
    {
        .OCMode = LL_TIM_OCMODE_PWM1,           // Change to PWM mode
        .OCState = LL_TIM_OCSTATE_ENABLE,       // Enable the output
        .CompareValue = (LL_TIM_GetAutoReload(TIM1) + 1) / 2, // Mid-point for center-aligned
        .OCPolarity = LL_TIM_OCPOLARITY_HIGH,   // Rising edge at compare match
        .OCIdleState = LL_TIM_OCIDLESTATE_LOW,
    };
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &ch4_tim1_OCInit);
 
    // Set as trigger output (for ADC synchronization)
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);
    
    LL_ADC_INJ_InitTypeDef adc1InjInitStruct = {
        .TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
        .SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS,
        .SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE,
        .TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT
    };                                                      

    // Initialize ADC injected group for time-critical measurements
    LL_ADC_INJ_Init(ADC1, &adc1InjInitStruct);

    //ADC CONFIGURATIONS AND LINKAGE TO TIM1 TRGO
    LL_ADC_INJ_StartConversionExtTrig(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);

    // Configure injected group sequence (PWM-synchronized measurements)
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, MCADCPWM3P_VSU_ADC_CHANNEL);  // Phase A Voltage
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, MCADCPWM3P_VSV_ADC_CHANNEL);  // Phase B Voltage
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_3, MCADCPWM3P_VSW_ADC_CHANNEL);  // Phase C Voltage

    // Set sampling times for all PWM Scan channels (injected group)
    LL_ADC_SetChannelSamplingTime(ADC1, MCADCPWM3P_VSU_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_15CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, MCADCPWM3P_VSV_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_15CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, MCADCPWM3P_VSW_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_15CYCLES);

    // Enable ADC interrupts
    LL_ADC_EnableIT_JEOS(ADC1);  // Injected sequence end interrupt    
}

//================================================================================================================
// ADC Initialization
//================================================================================================================
static void MCADCPWM3P_adcInit(const MCADCPWM3P_Config &cfg)
{
    __HAL_RCC_ADC1_CLK_ENABLE();
    LL_ADC_CommonInitTypeDef adc1CommonInitStruct = {
        .CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV2
        #ifdef LL_ADC_MULTI_INDEPENDENT
        .Multimode = LL_ADC_MULTI_INDEPENDENT,
        .MultiDMATransfer = LL_ADC_MULTI_REG_DMA_EACH_ADC,
        .MultiTwoSamplingDelay = LL_ADC_MULTI_TWOSAMPLINGDELAY_5CYCLES
        #endif
    };

    LL_ADC_InitTypeDef adc1InitStruct = {
        .Resolution = LL_ADC_RESOLUTION_12B,
        .DataAlignment = LL_ADC_DATA_ALIGN_RIGHT,
        .SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE
    };

    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &adc1CommonInitStruct);
    LL_ADC_Init(ADC1, &adc1InitStruct);

    // Enable ADC
    if (LL_ADC_IsEnabled(ADC1) == 0) 
    {
        LL_ADC_Enable(ADC1);
    }
}

void MCADCPWM3P_init(const MCADCPWM3P_Config &cfg)
{
    MCADCPWM3P_adc_gpioInit();
    MCADCPWM3P_pwm_gpioInit();
    MCADCPWM3P_tim1Init(cfg);
    MCADCPWM3P_tim1InterruptInit(cfg);
    MCADCPWM3P_I1.dutyMax_q15 = static_cast<uint16_t>((cfg.dutyMax_perc / 100.0f) * 0x7FFF);
    MCADCPWM3P_I1.dutyMin_q15 = static_cast<uint16_t>((cfg.dutyMin_perc / 100.0f) * 0x7FFF);

    MCADCPWM3P_adcInit(cfg);
    MCADCPWM3P_adc_calibrate();

    MCADCPWM3P_adcReg_dmaInit(cfg);
    MCADCPWM3P_adc_interruptInit(cfg);
    MCADCPWM3P_adcReg_ScanModeInit();

    //We cant support any runtime sampling mode except trapezoidal without any current sensors)
    MCADCPWM3P_adcPwm_TrapScanMode();
}

#define SATURATE(v , min , max)(( v > max)?max: ((v < min)?0:v))

void MCADCPWM3P_pwm_write(int16_t dutyU_q15, int16_t dutyV_q15, int16_t dutyW_q15)
{
    //MAXIMALLY optimized to around 110 Clock cycles max (1.3uS @ 84MHz)
    // AT 0% duty and positive duty (phase active or phase low, outputs are enabled)
    constexpr uint32_t U_MASK = (TIM_CCER_CC1NE);
    constexpr uint32_t V_MASK = (TIM_CCER_CC2NE);
    constexpr uint32_t W_MASK = (TIM_CCER_CC3NE);

    

    //Acroding to RM0368  , When OCCR=1   , (OCxE = 1, OCxNE = 1 (Complementary pwm with dead time))(OCxE = 1, OCxNE = 0 (OCNE is forced inactive), OCxE = OCxREF)
    // States we implement for pwm modes (Complementary PWM (OCxE , OCxNE , PWM1) , Forced low phase (OCxE = 1 , OCxNE = 1, FORCED INACTIVE), Floating phase  (OCxE = 1 , OCxNE = 0, FORCED INACTIVE))
    // For duty > 0: PWM mode (0b110)
    // For duty == 0: Force inactive (0b100)  
    // For duty < 0: Mode doesn't matter (outputs disabled)
    
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
    
    uint32_t ccer_shadow = TIM1->CCER;
    uint32_t ccmr1_shadow = TIM1->CCMR1;
    uint32_t ccmr2_shadow = TIM1->CCMR2;
    ccmr1_shadow &= ~(OC1M_MASK | OC2M_MASK);  // Clear OC1M bits
    ccmr2_shadow &= ~OC3M_MASK;  // Clear OC3M bits

    // ===== PHASE U (Channel 1) =====
    if(dutyU_q15 > 0){
        const uint32_t compareU = ((uint32_t)SATURATE(dutyU_q15, MCADCPWM3P_I1.dutyMin_q15, MCADCPWM3P_I1.dutyMax_q15) * MCADCPWM3P_I1.timerMax_q15) >> 15;
        ccer_shadow |= U_MASK;
        ccmr1_shadow |= OC1M_PWM;    // Set PWM mode
        TIM1->CCR1 = compareU;
    }else if(dutyU_q15 == 0){
        ccer_shadow |= U_MASK;
        ccmr1_shadow |= OC1M_INACTIVE; // Set force inactive mode
    }else{
        ccer_shadow &= ~U_MASK;
        ccmr1_shadow |= OC1M_INACTIVE; // Set force inactive mode
    }

    // ===== PHASE V (Channel 2) =====
    if(dutyV_q15 > 0){
        const uint32_t compareV = ((uint32_t)SATURATE(dutyV_q15, MCADCPWM3P_I1.dutyMin_q15, MCADCPWM3P_I1.dutyMax_q15) * MCADCPWM3P_I1.timerMax_q15) >> 15;
        ccer_shadow |= V_MASK;
        ccmr1_shadow |= OC2M_PWM;    // Set PWM mode
        TIM1->CCR2 = compareV;
    }else if(dutyV_q15 == 0){
        ccer_shadow |= V_MASK;
        ccmr1_shadow |= OC2M_INACTIVE; // Set force inactive mode
    }else{
        ccer_shadow &= ~V_MASK;
        ccmr1_shadow |= OC2M_INACTIVE; // Set force inactive mode
    }

    // ===== PHASE W (Channel 3) =====
    if(dutyW_q15 > 0){
        const uint32_t compareW = ((uint32_t)SATURATE(dutyW_q15, MCADCPWM3P_I1.dutyMin_q15, MCADCPWM3P_I1.dutyMax_q15) * MCADCPWM3P_I1.timerMax_q15) >> 15;
        ccer_shadow |= W_MASK;
        ccmr2_shadow |= OC3M_PWM;    // Set PWM mode
        TIM1->CCR3 = compareW;
    }else if(dutyW_q15 == 0){
        ccer_shadow |= W_MASK;
        ccmr2_shadow |= OC3M_INACTIVE; // Set force inactive mode
    }else{
        ccer_shadow &= ~W_MASK;
        ccmr2_shadow |= OC3M_INACTIVE; // Set force inactive mode
    }

    // ===== WRITE ALL SHADOW REGISTERS =====
    TIM1->CCER = ccer_shadow;
    TIM1->CCMR1 = ccmr1_shadow;
    TIM1->CCMR2 = ccmr2_shadow;
    
    // ===== GENERATE UPDATE EVENT =====
    TIM1->EGR |= TIM_EGR_COMG;
}

//======================================================================
// ReadPhase Voltages and Currents
//======================================================================
static inline void MCADCPWM3P_adcPwm_TrapRead(int32_t scanData[6])
{
    //==================================================================
    //VoltageMeasurements based on Phase State
    //==================================================================
    scanData[0] = U_VS_uV_TO_mV(MCADCPWM3P_ADC_TO_uV(
                                        LL_ADC_INJ_ReadConversionData32(ADC1, LL_ADC_INJ_RANK_1)
                                    ));
    scanData[1] = V_VS_uV_TO_mV(MCADCPWM3P_ADC_TO_uV(
                                        LL_ADC_INJ_ReadConversionData32 (ADC1, LL_ADC_INJ_RANK_2)
                                    ));
    scanData[2] = W_VS_uV_TO_mV(MCADCPWM3P_ADC_TO_uV(
                                        LL_ADC_INJ_ReadConversionData32 (ADC1, LL_ADC_INJ_RANK_3)
                                    ));

    scanData[3] = 0.0f;
    scanData[4] = 0.0f;
    scanData[5] = 0.0f;
}


void MCADCPWM3P_adcPwm_read(int32_t scanData[6]){
    return MCADCPWM3P_adcPwm_TrapRead(scanData);
}


extern "C" {
    void ADC_IRQHandler(void)
    {
    if (LL_ADC_IsActiveFlag_JEOS(ADC1)) {
        LL_ADC_ClearFlag_JEOS(ADC1);
        if (adcPwm_postScanCallback != nullptr) 
        {
            adcPwm_postScanCallback();
        }
    }
    }


    void TIM1_UP_TIM10_IRQHandler(void)
    {
        if (LL_TIM_IsActiveFlag_UPDATE(TIM1)) {
            LL_TIM_ClearFlag_UPDATE(TIM1);
            static volatile uint8_t pwm_cnt = 0;
            pwm_cnt+=1;
            // Check if we're at peak (counting down) or valley (counting up)
            // Peak interrupt - counting down phase just started
            // This is typically where you want the mid-PWM trigger
            if(pwm_cnt >= uint8_t(MCADCPWM3P_PWMFREQ_ADCREGSCAN_RATIO*2)){
                MCADCPWM3P_adcReg_startConv();
                pwm_cnt = 0;
            }
        }
    }
}
