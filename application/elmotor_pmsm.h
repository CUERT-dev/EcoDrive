#pragma once
#include "core/elmath.h"
#include "core/elcore.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_conf.h"
#include "sys.h"
#include "arm_math.h"
//===================================================
// CONFIG
//==================================================

#ifdef __cplusplus
extern "C" {
#endif

#define PWM_FREQ                   20000
#define XMCPWM_TICKFREQ            PWM_FREQ
#define XMCPWM_TICKPERIOD_US       (1000000.0/XMCPWM_TICKFREQ)
#define XMCPWM_TICKPERIOD_MS       (XMCPWM_TICKPERIOD_US/1000.0)
#define XMCPWM_US_TO_TICKS(us)     (us/XMCPWM_TICKPERIOD_US)
#define XMCPWM_MS_TO_TICKS(ms)     (XMCPWM_US_TO_TICKS(ms*1000))
#define XMCPWM_TICKS_TO_US(ticks)  (ticks * XMCPWM_TICKPERIOD_US)
#define XMCPWM_TICKS_TO_MS(ticks)  (ticks * XMCPWM_TICKPERIOD_MS)

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
    eldriver_mc3p_handle_t mc3p;
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

    float bus_voltage_q31;
    q15_t mc3p_trap_duty_q15;
    eldriver_mc3p_sector_t mc3p_sector;
    elmotor_dir_t dir;
    elmotor_pmsm_state state;
} elmotor_pmsm_handle_t;


extern elmotor_pmsm_handle_t motor_c;
void elmotor_pmsm_init(elmotor_pmsm_handle_t *cp, elmotor_pmsm_stup_config_t stup_cfg);




#include "core/elcore.h"

#define SAMPLE_LEN 5
#define SAMPLES_PER_FRAME 12
#define FRAME_BUFFER_COUNT 8 
#define FRAME_BUFFER_NOTIFY_THRESHOLD 6

typedef float pwmSample_t[SAMPLE_LEN];
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