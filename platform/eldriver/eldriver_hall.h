#ifndef TIM2_UTIL_H
#define TIM2_UTIL_H
#include <stdint.h>
#include "eldriver_conf.h"

typedef struct{


}eldriver_hall_config_t;

typedef struct{
    eldriver_hall_config_t config;
}eldriver_hall_handle_t;

void eldriver_hall1_init(eldriver_hall_handle_t* handle);
void eldriver_hall1_triggerSw(eldriver_hall_handle_t* handle);
void eldriver_hall1_setComDelay_uS(eldriver_hall_handle_t* handle, uint32_t COM_delay_uS);
void eldriver_hall1_setComCallback(eldriver_hall_handle_t* handle, void (*callback)(void));

inline uint8_t eldriver_hall1_read()
{
    // JUST SIMPLE GPIO READ
    #ifdef HALL1_HARDWARE
    extern volatile uint8_t hall1_value;
    return hall1_value;
    #endif

    return 0;
}

//Period between edges/Changes
inline uint16_t eldriver_hall1_getPeriod_uS()
{
    extern volatile uint16_t hall1_period_uS;
    return hall1_period_uS;
}

#endif


