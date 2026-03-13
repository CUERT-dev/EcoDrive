#pragma once
#include "sensor.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_core.h"
#include "elmotor_pmsm.h"
#include <stdbool.h>


void bemfzf_init(bemfzc_t *h, float threshold_low_V, float threshold_high_V, float phase_delay, uint16_t tick_freq)
{
    h->threshold_high = ELDRIVER_MC3P_FLOAT_TO_VS(threshold_high_V);
    h->threshold_low = ELDRIVER_MC3P_FLOAT_TO_VS(threshold_low_V);
    h->phase_delay = phase_delay;
    h->last_zc_tick = 0;
    h->tick_freq = tick_freq;
    eldriver_comDelay_init();
    h->state = false;
}

void bemfzc_update(bemfzc_t *h, int32_t bemf, uint32_t ticks, uint8_t dir)
{
    bool rising  = (bemf > h->threshold_high && h->state == false);
    bool falling = (bemf < h->threshold_low && h->state == true);
    
    if(rising || falling){
        if(rising){h->state = true;}else{h->state = false;}
        h->elec_angle_q31 += dir * INT32_MAX/6;
        h->elec_speed = ((XCPWM_TICKFREQ/(ticks - h->last_zc_tick))/6) * 2 * M_PI;
        uint32_t delay_us = ((h->phase_delay + M_PI/6)/ h->elec_speed)*1e6;
        h->last_zc_tick = ticks;
        eldriver_comDelay_setComDelay_uS(delay_us);
    }
}

void bemfzc_takeover(bemfzc_t *h, void(*cb)(void))
{
    eldriver_comDelay_setComCallback(cb);
}

int32_t bemfzc_elec_angle_q31(bemfzc_t *h)
{
    return h->elec_angle_q31;
}

float bemfzc_elec_speed(bemfzc_t *h)
{
    return h->elec_speed + 100;
}

void bemfzc_reset(bemfzc_t *h)
{
    h->state = false;
    h->elec_angle_q31 = 0;   
}

void hall1_update()
{
}
