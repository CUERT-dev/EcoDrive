#ifndef MCADCPWM3P_H
#define MCADCPWM3P_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    ELDRIVER_MC3P_SECTOR_FLOAT = 0,
    ELDRIVER_MC3P_SECTOR_TRAP1,
    ELDRIVER_MC3P_SECTOR_TRAP2,
    ELDRIVER_MC3P_SECTOR_TRAP3,
    ELDRIVER_MC3P_SECTOR_TRAP4,
    ELDRIVER_MC3P_SECTOR_TRAP5,
    ELDRIVER_MC3P_SECTOR_TRAP6,
    ELDRIVER_MC3P_SECTOR_SVM1,
    ELDRIVER_MC3P_SECTOR_SVM2,
    ELDRIVER_MC3P_SECTOR_SVM3,
    ELDRIVER_MC3P_SECTOR_SVM4,
    ELDRIVER_MC3P_SECTOR_SVM5,
    ELDRIVER_MC3P_SECTOR_SVM6
} eldriver_mc3p_sector_t;

typedef enum {
    ELDRIVER_MC3P_MODE_NONE = 0,
    ELDRIVER_MC3P_MODE_TRAP,
    ELDRIVER_MC3P_MODE_SVM 
} eldriver_mc3p_mode_t;

#define IS_SVM_SECTOR(sector) (sector >= ELDRIVER_MC3P_SECTOR_SVM1 && sector <= ELDRIVER_MC3P_SECTOR_SVM6)
#define IS_TRAP_SECTOR(sector) (sector >= ELDRIVER_MC3P_SECTOR_TRAP1 && sector <= ELDRIVER_MC3P_SECTOR_TRAP6)

typedef enum {
    ELDRIVER_MC3P_PHASE_FLOAT = 0,
    ELDRIVER_MC3P_PHASE_H_PWM,
    ELDRIVER_MC3P_PHASE_L_PWM,
    ELDRIVER_MC3P_PHASE_COMP,
    ELDRIVER_MC3P_PHASE_H_ON,
    ELDRIVER_MC3P_PHASE_L_ON
} eldriver_mc3p_phase_state_t;  

typedef struct{
    uint32_t pwm_Hz;
    uint32_t deadtime_nS;
    float duty_max;
    float duty_min;
}eldriver_mc3p_config_t;

typedef struct{
    eldriver_mc3p_config_t config;
    int16_t adc_to_uV;
    float adc_ref_V;
    float internal_ref_V;
    volatile eldriver_mc3p_mode_t mode;
    volatile eldriver_mc3p_sector_t sector_last;
    uint32_t timer_max_q15;
    uint16_t duty_max_q15;
    uint16_t duty_min_q15;
    uint16_t dutyu_q15;
    uint16_t dutyv_q15;
    uint16_t dutyw_q15;
}eldriver_mc3p_handle_t;

typedef struct
{
    uint32_t vbus_q31;
    uint32_t cu_q31;
    uint32_t cv_q31;
    uint32_t cw_q31;
} eldriver_mc3p_svm_data_t;

typedef struct{
    uint32_t vbus_q31;
    uint32_t vbemf_q31;
    uint32_t cbus_q31;
} eldriver_mc3p_trap_data_t;



//TODO  FINISH ADC IMPLEMENTATION FOR 1)TRAP & 2)SVM

void eldriver_mc3p_init(eldriver_mc3p_handle_t *h);

void eldriver_mc3p_bg_startConv(eldriver_mc3p_handle_t *h);
uint8_t eldriver_mc3p_bg_channels(eldriver_mc3p_handle_t *h);
uint8_t eldriver_mc3p_read_bg(eldriver_mc3p_handle_t *h, float* scanData);
uint8_t eldriver_mc3p_bg_isReady(eldriver_mc3p_handle_t *h);
void eldriver_mc3p_read_sync(eldriver_mc3p_handle_t *h, void* scanData);

void eldriver_mc3p_write_phase_state(eldriver_mc3p_handle_t *h, eldriver_mc3p_phase_state_t state_u, eldriver_mc3p_phase_state_t state_v, eldriver_mc3p_phase_state_t state_w);
void eldriver_mc3p_write_phase_duty(eldriver_mc3p_handle_t *h, uint16_t duty_u_q15, uint16_t duty_v_q15, uint16_t duty_w_q15);

void eldriver_mc3p_write_float(eldriver_mc3p_handle_t *h);
void eldriver_mc3p_write_trap(eldriver_mc3p_handle_t *h, eldriver_mc3p_sector_t sector, uint16_t duty_q15);
void eldriver_mc3p_write_svm(eldriver_mc3p_handle_t *h, int16_t alpha_q15, int16_t beta_q15);

__attribute__((weak)) void eldriver_xmc3p_tickerCallback(void);
__attribute__((weak)) void eldriver_mc3p_sync_postScanCallback(void);

#ifdef __cplusplus
}
#endif


#endif