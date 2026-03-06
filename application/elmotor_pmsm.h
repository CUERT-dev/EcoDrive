#pragma once
#include "core/elmath.h"
#include "core/elcore.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_conf.h"
#include "eldriver/eldriver_hall.h"
#include "sys.h"
#include "arm_math.h"
//===================================================
// CONFIG
//==================================================

#ifdef __cplusplus
extern "C" {
#endif

#define PWM_FREQ                   40000
#define BEMF_THRESHOLD_LOW         0.1
#define BEMF_THRESHOLD_HIGH        0.3
#define COMMUTATION_PHASE_DELAY    0
#define STUP_BEMFZC_ERROR_MARGIN    0.05 
#define STUP_BEMFZC_GOOD_EST_COUNT        10

#define XCPWM_TICKFREQ            PWM_FREQ
#define XCPWM_TICKPERIOD_US       (1000000.0/XCPWM_TICKFREQ)
#define XCPWM_TICKPERIOD_MS       (XCPWM_TICKPERIOD_US/1000.0)
#define XCPWM_US_TO_TICKS(us)     (us/XCPWM_TICKPERIOD_US)
#define XCPWM_MS_TO_TICKS(ms)     (XCPWM_US_TO_TICKS(ms*1000))
#define XCPWM_TICKS_TO_US(ticks)  (ticks * XCPWM_TICKPERIOD_US)
#define XCPWM_TICKS_TO_MS(ticks)  (ticks * XCPWM_TICKPERIOD_MS)

#define STUP_TABLE_SIZE 4

typedef struct
{
    uint16_t align_duration_ms;
    eldriver_mc3p_sector_t align_sector;
    float bus_V;
    float align_V;
    float time_mS[STUP_TABLE_SIZE];
    float volt_V[STUP_TABLE_SIZE];
    float freq_Hz[STUP_TABLE_SIZE];
} elmotor_pmsm_stup_config_t;

typedef enum
{
    STUP_STAGE_STBEMF_RESET = 0,
    STUP_STAGE_STBEMF_ALIGN,
    STUP_STAGE_STBEMF_RAMP,
    STUP_STAGE_STBEMF_SWITCHOVER
} pmsm_stup_stage_t;

typedef enum
{
    ELMOTOR_DIR_FORWARD = 0,
    ELMOTOR_DIR_BACKWARD
} elmotor_dir_t;


typedef enum{
    ELMOTOR_IDLE = 0,
    ELMOTOR_STUP_TRAPBEMF,
    ELMOTOR_CL_TRAP
}elmotor_pmsm_state;


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
    eldriver_mc3p_t mc3p;

    bool initialized;
    union{
        eldriver_mc3p_svm_data_t svm;
        eldriver_mc3p_trap_data_t trap;
    }mc3p_sync_data;

    uint32_t xmc_ticks;

    elmotor_pmsm_stup_config_t stup_cfg;
    elcore_swttimer_t stup_stage_timer;
    elcore_swttimer_t stup_comm_timer;
    uint32_t          stup_comm_ticks;
    pmsm_stup_stage_t stup_stage_last;
    pmsm_stup_stage_t stup_stage_current;
    uint8_t           stup_ramp_idx;
    float             stup_est_elec_speed;
    uint8_t           stup_good_est_count;

    // q31_t vbus_q31;
    // q31_t vbemf_q31;
    // q31_t iu_q31;
    // q31_t iv_q31;
    // q31_t iw_q31;
    
    float elec_speed;
    q15_t mc3p_alpha_q15;
    q15_t mc3p_beta_q15;
    q15_t mc3p_trap_duty_q15;
    eldriver_mc3p_sector_t mc3p_sector;

    elmotor_dir_t dir;
    elmotor_pmsm_state state;

    bemfzc_t bemfzc;
} elmotor_pmsm_t;


extern elmotor_pmsm_t motor_c;
void elmotor_pmsm_init(elmotor_pmsm_t *cp, elmotor_pmsm_stup_config_t stup_cfg);




#include "core/elcore.h"

#define SAMPLE_LEN 5
#define SAMPLES_PER_FRAME 24
#define FRAME_BUFFER_COUNT 8 
#define FRAME_BUFFER_NOTIFY_THRESHOLD 6

typedef int16_t pwmSample_t[SAMPLE_LEN];
typedef struct
{
    uint32_t sample_counter;
    pwmSample_t samples[SAMPLES_PER_FRAME]; // 5 floats per sample as per schema
}PwmDataFrame_t;


typedef struct
{
    PwmDataFrame_t frames[FRAME_BUFFER_COUNT];
    elcore_rstream_t buffer;
    PwmDataFrame_t* currentFrame;
    uint32_t frame_sample_idx;
    uint32_t sample_count;
    uint32_t overflowCount; // Count of how many times data was lost due to overflow
}pwmDataBuffer_t;




void pwmDataBuffer_init(pwmDataBuffer_t *cp);
pwmSample_t *pwmDataBuffer_sample(pwmDataBuffer_t *cp, uint8_t *len);
void pwmDataBuffer_pushSample(pwmDataBuffer_t *cp);
bool pwmDataBuffer_readFrame(pwmDataBuffer_t *cp, PwmDataFrame_t **outFrame);
extern pwmDataBuffer_t pwmDataBuffer;


#ifdef __cplusplus
}
#endif