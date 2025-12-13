
#include "MCADCPWM3P_common.h"
#include "../MCADCPWM3P.h"

//================================================================
// MCU Family Dependent LAYER GLOBALS and Definitions
//================================================================

//MCU LINE SPECIFIC DEFINES
#define VREFIN_CAL_ADDR (uint16_t *)0x1FFF7A2A
#define VREFIN_CAL_VOLTAGE 3.3f


void(*adcPwm_postScanCallback)();

float adcVoltageRef_volt;
float internalRefVoltage_volt;
const float externalRefVoltage_volt = VREFEXT_CAL_VOLTAGE;
int16_t adc_to_uV;

//======================================================
//DMA static definitions for ADC
uint16_t MCADCPWM3P_regScanData[2][MCADCPWM3P_REGSCAN_CHANNEL_NUM];
int32_t MCADCPWM3P_regScanData_uV[MCADCPWM3P_REGSCAN_CHANNEL_NUM];
static volatile uint8_t MCADCPWM3P_regScanActiveBuffer = 0; // 0 or 1
bool isReady = false;

//NEVER FORGET 
//THAT YOU EVER CHANGE THESE VALUES FOR DMA YOU STILL HAVE TO CHANGE IMPLEMENTATION AND IRQ APPROPIATELY
#define MCADCPWM3P_DMA_REGSCAN_STREAM LL_DMA_STREAM_0
#define MCADCPWM3P_DMA_INSTANCE DMA2
#define MCADCPWM3P_DMA_CHANNEL LL_DMA_CHANNEL_0


//=========================================
// Adc Single Sample Function
//==========================================
uint16_t Adc1_Sample_Single_Channel_Temporary(uint32_t channel)
{
    // Save current configuration
    uint32_t old_sqr1 = ADC1->SQR1;
    uint32_t old_sqr2 = ADC1->SQR2;
    uint32_t old_sqr3 = ADC1->SQR3;
    
    // Configure for single conversion
    // Configure for single conversion USING LL FUNCTION
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE); // 1 conversion
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, channel);   // Handles conversion 
    
    // Start conversion
    LL_ADC_REG_StartConversionSWStart(ADC1);
    while (!LL_ADC_IsActiveFlag_EOCS(ADC1));
    uint16_t result = LL_ADC_REG_ReadConversionData12(ADC1);
    
    // Restore original configuration
    ADC1->SQR1 = old_sqr1;
    ADC1->SQR2 = old_sqr2;
    ADC1->SQR3 = old_sqr3;
    
    return result;
}

//=========================================
// AdcCalibrate Function
//==========================================
void MCADCPWM3P_adc_calibrate()
{
    //Enable VREFINT and internalTEMP sensor
    ADC->CCR |= ADC_CCR_TSVREFE_Msk;
    #ifndef ADC_EXTERNAL_VREF

    // Wait for internal references to stabilize (IMPORTANT!)
    volatile uint32_t wait = 10000;
    while(wait){wait = wait - 1;};

    // Set longer sampling time for VREFINT
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_480CYCLES);

    // Sample VREFINT multiple times for accuracy
    uint32_t sum = 0;
    const uint8_t num_samples = 16;

    //Discard first sample

    Adc1_Sample_Single_Channel_Temporary(LL_ADC_CHANNEL_VREFINT);
    for (int i = 0; i < num_samples; i++) {
        sum += Adc1_Sample_Single_Channel_Temporary(LL_ADC_CHANNEL_VREFINT);
    }
    uint32_t inRef_div_adcRef = sum / num_samples;

    internalRefVoltage_volt = (*VREFIN_CAL_ADDR / float((1<<ADC_RESOLUTION_BITS) - 1) ) * VREFIN_CAL_VOLTAGE;

    // Calculate actual ADC reference voltage (your supply voltage)
    adcVoltageRef_volt = internalRefVoltage_volt * ((1<<ADC_RESOLUTION_BITS) - 1) / (float)inRef_div_adcRef;
    #else 

    adcVoltageRef_volt = externalRefVoltage_volt;
    #endif

    adc_to_uV = float( adcVoltageRef_volt * 1000000 ) / float((1<<ADC_RESOLUTION_BITS) - 1);
}


//================================================================================================================
// RegScanModes for Slow Monitoring and application layer data aquisition (Temperature Readings , Throttle , etc)
//================================================================================================================

void MCADCPWM3P_adcReg_ScanModeInit()
{
    LL_ADC_REG_InitTypeDef adc1RegInitStruct = {    
        .TriggerSource = LL_ADC_REG_TRIG_SOFTWARE,
        .SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS,
        .SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_3RANKS,
        .ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS,
        .DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED
    };

    // Initialize ADC regular group for slow measurements
    LL_ADC_REG_Init(ADC1, &adc1RegInitStruct);

    // Configure regular group sequence (slow monitoring channels)
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_18); // Throttle
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_18); // Chip Junction Temp
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_18); // Chip Junction Temp

    // Set sampling times for all regular Scan channels (regular group)
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_18, LL_ADC_SAMPLINGTIME_56CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_18, LL_ADC_SAMPLINGTIME_56CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_18, LL_ADC_SAMPLINGTIME_56CYCLES);
}


//================================================================================================================
// DMA Initialization
//================================================================================================================
void MCADCPWM3P_adcReg_dmaInit(const MCADCPWM3P_Config &cfg)
{
    // Enable DMA2 clock (ADC1 is connected to DMA2 on STM32F4)
    __HAL_RCC_DMA2_CLK_ENABLE();
    
    // =================================================================
    // DMA for ADC1 Regular Scan (Slow Monitoring Channels)
    // =================================================================
    
    // Configure DMA channel first (ADC1 uses Channel 0 for Stream 0)
    LL_DMA_SetChannelSelection(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM, MCADCPWM3P_DMA_CHANNEL);
    
    // Configure DMA parameters for ADC1 regular conversions
    LL_DMA_SetDataTransferDirection(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM, LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetMode(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_DisableFifoMode(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM);
    
    // Set peripheral address (ADC1 data register)
    LL_DMA_SetPeriphAddress(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM, (uint32_t)&(ADC1->DR));
    
    // Set memory address (double buffer start)
    LL_DMA_SetMemoryAddress(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM, (uint32_t)MCADCPWM3P_regScanData[0]);
    
    // Set data length (total samples across both buffers)
    LL_DMA_SetDataLength(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM, 
                        MCADCPWM3P_REGSCAN_CHANNEL_NUM);
    
    // Enable DMA transfer complete and half transfer interrupts
    LL_DMA_EnableIT_TC(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM);
    LL_DMA_EnableIT_HT(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM);
    
    // Enable DMA stream
    LL_DMA_EnableStream(MCADCPWM3P_DMA_INSTANCE, MCADCPWM3P_DMA_REGSCAN_STREAM);
}


//================================================================================================================
// Interrupt Initialization
//================================================================================================================
void MCADCPWM3P_adc_interruptInit(const MCADCPWM3P_Config &cfg){
    // Regular scan DMA interrupt - use LL macros for portability
    NVIC_SetPriority(DMA2_Stream0_IRQn, 5);  // Medium priority for slow monitoring
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    // ADC Injected End of Conversion (JEOC) interrupt - Higher priority for time-critical measurements
    NVIC_SetPriority(ADC_IRQn, 0);           // Higher priority than DMA for PWM-synchronized measurements
    NVIC_EnableIRQ(ADC_IRQn);
    
    // Enable JEOS (Injected End Of Sequence) interrupt in ADC
    LL_ADC_EnableIT_JEOS(ADC1);
}

//==================================================================================
// Read Regular Slow channels (are always oversampled and prestored in DMA buffer)
//==================================================================================
const int32_t* MCADCPWM3P_adcReg_read(void)
{
    uint8_t readBuffer = 1 - MCADCPWM3P_regScanActiveBuffer; // Always read from inactive buffer
    for(int i = 0; i < MCADCPWM3P_REGSCAN_CHANNEL_NUM; i++){
        MCADCPWM3P_regScanData_uV[i] = MCADCPWM3P_ADC_TO_uV(MCADCPWM3P_regScanData[readBuffer][i]);
    }
    isReady = false;
    return MCADCPWM3P_regScanData_uV;
}


void MCADCPWM3P_adcReg_startConv()
{
    isReady = false;
    LL_ADC_REG_StartConversionSWStart(ADC1);
}

bool MCADCPWM3P_adcReg_isReady(void)
{
    return isReady;
}

void MCADCPWM3P_adcPwm_registerPostScanCallback(void (*callback)()) 
{
    adcPwm_postScanCallback = callback;
}

extern "C"
{
    void DMA2_Stream0_IRQHandler(void)
      {
        if (LL_DMA_IsActiveFlag_HT0(MCADCPWM3P_DMA_INSTANCE)) {
            LL_DMA_ClearFlag_HT0(MCADCPWM3P_DMA_INSTANCE);
            // Half transfer complete - First Half samples Done
            MCADCPWM3P_regScanActiveBuffer = 1;
            isReady = true;
        }
        
        if (LL_DMA_IsActiveFlag_TC0(MCADCPWM3P_DMA_INSTANCE)) {
            LL_DMA_ClearFlag_TC0(MCADCPWM3P_DMA_INSTANCE);
            // Full transfer complete - Second Half Samples Done
            MCADCPWM3P_regScanActiveBuffer = 0;
            isReady = true;
        }
    }
}


