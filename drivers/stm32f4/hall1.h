#ifndef TIM2_UTIL_H
#define TIM2_UTIL_H
#include <stdint.h>
#include "driver_bsp.h"


void hall1_init();
void hall1_triggerSWChange(void);
void hall1_setCOMDelay(uint32_t COM_delay_uS);
void hall1_setCOMEVENTCallback(void (*callback)(void));

inline uint8_t hall1_read()
{
    // JUST SIMPLE GPIO READ
    #ifdef HALL1_HARDWARE
    extern volatile uint8_t hall1_value;
    return hall1_value;
    #endif

    return 0;
}

//Period between edges/Changes
inline uint16_t hall1_getPeriod_uS()
{
    extern volatile uint16_t hall1_period_uS;
    return hall1_period_uS;
}

#endif


