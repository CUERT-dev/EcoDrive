#include "eldriver_mc3p.h"



void eldriver_mc3p_init(eldriver_mc3p_t *h,const float scales[MC3P_SYNC_CHANNELS][2]){}
void eldriver_mc3p_setScales(eldriver_mc3p_t *h,const float scales[MC3P_SYNC_CHANNELS][2]){}
void eldriver_mc3p_bg_startConv(eldriver_mc3p_t *h){}
uint8_t eldriver_mc3p_bg_channels(eldriver_mc3p_t *h){}
uint8_t eldriver_mc3p_read_bg(eldriver_mc3p_t *h, float* scanData){}
uint8_t eldriver_mc3p_bg_isReady(eldriver_mc3p_t *h){}
void eldriver_mc3p_read_sync(eldriver_mc3p_t *h, void* scanData){}

void eldriver_mc3p_write_phase_state(eldriver_mc3p_t *h, eldriver_mc3p_phase_state_t state_u, eldriver_mc3p_phase_state_t state_v, eldriver_mc3p_phase_state_t state_w){}
void eldriver_mc3p_write_phase_duty(eldriver_mc3p_t *h, uint16_t duty_u_q15, uint16_t duty_v_q15, uint16_t duty_w_q15){}

void eldriver_mc3p_write_float(eldriver_mc3p_t *h){}
void eldriver_mc3p_write_trap(eldriver_mc3p_t *h, eldriver_mc3p_sector_t sector, uint16_t duty_q15){}
void eldriver_mc3p_write_svm(eldriver_mc3p_t *h, int16_t alpha_q15, int16_t beta_q15){}