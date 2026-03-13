#pragma once
#include "eldriver/eldriver_conf.h"
#include "eldriver/eldriver_hall.h"

#ifndef ELDRIVER_HALL1_ENABLED
#define ELDRIVER_BEMFZC_ENABLED
#endif







#define BEMF_THRESHOLD_LOW         0.1
#define BEMF_THRESHOLD_HIGH        0.3
#define COMMUTATION_PHASE_DELAY    0


typedef struct{
    int32_t threshold_low;
    int32_t threshold_high;
    int32_t elec_angle_q31;
    uint32_t last_zc_tick;
    uint16_t tick_freq;
    float elec_speed;
    float phase_delay;
    uint8_t state;
} bemfzc_t;

void bemfzf_init(bemfzc_t *h,float threshold_low_V, float threshold_high_V, float phase_delay, uint16_t tick_freq);
void bemfzc_update(bemfzc_t *h, int32_t bemf, uint32_t ticks, uint8_t dir);
void bemfzc_takeover(bemfzc_t *h, void(*cb)(void));
int32_t bemfzc_elec_angle_q31(bemfzc_t *h);
float bemfzc_elec_speed(bemfzc_t *h);
void bemfzc_reset(bemfzc_t *h);



typedef struct{


}hall_t;
void    hall1_update();
int32_t hall1_elec_angle_q31();

typedef union
{
    bemfzc_t bemf;
    hall_t   hall;
}pos_sensor_t;



// Compile-time position sensor dispatch (no function pointers)
static inline void pos_init(pos_sensor_t *pos_ctx, uint16_t tick_freq)
{
#ifdef ELDRIVER_HALL1_ENABLED
    (void)pos_ctx;
    (void)tick_freq;
    eldriver_hall1_init();
#else
    bemfzf_init(&pos_ctx->bemf, BEMF_THRESHOLD_LOW, BEMF_THRESHOLD_HIGH, COMMUTATION_PHASE_DELAY, tick_freq);
#endif
}

static inline void pos_update(pos_sensor_t *pos_ctx, int32_t bemf_q31, uint32_t ticks, uint8_t dir)
{
#ifdef ELDRIVER_HALL1_ENABLED
    (void)pos_ctx;
    (void)bemf_q31;
    (void)ticks;
    (void)dir;
    hall1_update();
#else
    bemfzc_update(&pos_ctx->bemf, bemf_q31, ticks, dir);
#endif
}

static inline float pos_elec_speed(pos_sensor_t *pos_ctx)
{
#ifdef ELDRIVER_HALL1_ENABLED
    (void)pos_ctx;
    return eldriver_hall1_elec_speed();
#else
    return bemfzc_elec_speed(&pos_ctx->bemf);
#endif
}

static inline int32_t pos_elec_angle_q31(pos_sensor_t *pos_ctx)
{
#ifdef ELDRIVER_HALL1_ENABLED
    (void)pos_ctx;
    return eldriver_hall1_elec_angle_q31();
#else
    return bemfzc_elec_angle_q31(&pos_ctx->bemf);
#endif
}

static inline void pos_set_com_delay_uS(pos_sensor_t *pos_ctx, uint32_t delay_uS)
{
#ifdef ELDRIVER_HALL1_ENABLED
    eldriver_hall1_setComDelay_uS(delay_uS);
#else
    eldriver_comDelay_setComDelay_uS(delay_uS);
#endif
}

static inline void pos_set_com_callback(pos_sensor_t *pos_ctx, void (*callback)(void))
{
#ifdef ELDRIVER_HALL1_ENABLED
    eldriver_hall1_setComCallback(callback);
#else
    eldriver_comDelay_setComCallback(callback);
#endif
}