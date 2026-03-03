#include "elmotor_pmsm.h"
#include "eldriver/eldriver_mc3p.h"
#include "arm_math.h"


// For trap sectors specifically (1-6)
#define TRAP_INCREMENT(mc3p_sector, dir)((dir == ELMOTOR_DIR_FORWARD)?elmath_increment_roll(mc3p_sector, ELDRIVER_MC3P_SECTOR_TRAP1, ELDRIVER_MC3P_SECTOR_TRAP6):elmath_decrement_roll(mc3p_sector, ELDRIVER_MC3P_SECTOR_TRAP1, ELDRIVER_MC3P_SECTOR_TRAP6))

uint8_t started = 0;

elmotor_pmsm_handle_t motor_c;
void elmotor_pmsm_init(elmotor_pmsm_handle_t *cp, elmotor_pmsm_stup_config_t stup_cfg)
{
    cp->xmc_ticks = 0;
    cp->mc3p.config.pwm_Hz = PWM_FREQ;
    cp->mc3p.config.duty_max = 0.95;
    cp->mc3p.config.duty_min = 0.05;
    cp->mc3p.config.deadtime_nS = 1500;

    cp->stup_cfg    = stup_cfg;
    cp->stup_comm_ticks = 0;
    cp->mc3p_sector     = ELDRIVER_MC3P_SECTOR_FLOAT;
    cp->stup_comm_timer.tick_period = XMCPWM_TICKPERIOD_US;
    cp->stup_stage_timer.tick_period = XMCPWM_TICKPERIOD_US;
    cp->state = ELMOTOR_STUP_TRAPBEMF;
    cp->stup_stage_current = STUP_STAGE_STBEMF_RESET;
    eldriver_mc3p_init(&(cp->mc3p));
    started = 1;
}

void pmsm_idle(elmotor_pmsm_handle_t *cp)
{
    
}

void pmsm_stup_trapbemf(elmotor_pmsm_handle_t *cp)
{
    switch (cp->stup_stage_current)
    {
    case STUP_STAGE_STBEMF_RESET:
        cp->stup_stage_last    = STUP_STAGE_STBEMF_RESET;
        cp->stup_stage_current = STUP_STAGE_STBEMF_ALIGN;
    case STUP_STAGE_STBEMF_ALIGN:
        if(cp->stup_stage_current != cp->stup_stage_last)
        {
            //APPLY ALIGN VOLTAGE
            float32_t r = (float32_t)(cp->stup_cfg.align_V / cp->stup_cfg.bus_V);
            arm_float_to_q15(&r, &(cp->mc3p_trap_duty_q15), 1);
            cp->mc3p_sector = cp->stup_cfg.align_sector; 
            elcore_swttimer_reset(&(cp->stup_stage_timer), cp->xmc_ticks);
            cp->stup_stage_last = STUP_STAGE_STBEMF_ALIGN;
        }
        
        if(elcore_swttimer_timout(&(cp->stup_stage_timer), cp->xmc_ticks, XMCPWM_MS_TO_TICKS(cp->stup_cfg.align_duration_ms)))
        {
            cp->stup_stage_current = STUP_STAGE_STBEMF_RAMP;
        }
        else
        {
            break;
        }
    case STUP_STAGE_STBEMF_RAMP:
        if(cp->stup_stage_current != cp->stup_stage_last)
        {
            elcore_swttimer_reset(&(cp->stup_stage_timer), cp->xmc_ticks);
            elcore_swttimer_reset(&(cp->stup_comm_timer), cp->xmc_ticks);
            cp->stup_stage_last = STUP_STAGE_STBEMF_RAMP;
        }
        uint8_t comm = elcore_swttimer_timout(&cp->stup_comm_timer, cp->xmc_ticks, cp->stup_comm_ticks);
        if(cp->stup_ramp_idx < STUP_TABLE_SIZE - 1)
        {    
            //increment voltage
            float et_mS = XMCPWM_TICKS_TO_MS(elcore_swttimer_elapsed_ticks(&cp->stup_stage_timer, cp->xmc_ticks));
            float32_t v_norm = elmath_linearInterp(&cp->stup_cfg.volt_V[cp->stup_ramp_idx], &cp->stup_cfg.time_mS[cp->stup_ramp_idx], et_mS)/cp->stup_cfg.bus_V;
            arm_float_to_q15((&v_norm), &cp->mc3p_trap_duty_q15, 1);
            //increment frequency
            if(comm)
            {
                float32_t f = elmath_linearInterp(&cp->stup_cfg.freq_Hz[cp->stup_ramp_idx], &cp->stup_cfg.time_mS[cp->stup_ramp_idx], et_mS);
                cp->stup_comm_ticks = XMCPWM_MS_TO_TICKS(1000.0 / (f * 6));
            }
            if(et_mS > cp->stup_cfg.time_mS[cp->stup_ramp_idx + 1]){
                cp->stup_ramp_idx++;
                if(cp->stup_ramp_idx == STUP_TABLE_SIZE)
                { 
                    //cp->stup_stage_current = STUP_STAGE_STBEMF_SWITCHOVER;
                }
            }
        }else{
            

        }
        if(comm){
            cp->mc3p_sector = TRAP_INCREMENT(cp->mc3p_sector, cp->dir);   
            elcore_swttimer_reset(&(cp->stup_comm_timer), cp->xmc_ticks);
        }
        break;
    default:
        break;
    }
}

void pmsm_cl_trap(elmotor_pmsm_handle_t *cp)
{

}

void pmsm_break(elmotor_pmsm_handle_t *cp)
{

}

void eldriver_mc3p_sync_postScanCallback()
{
    if(started){
        eldriver_mc3p_read_sync(&motor_c.mc3p, &motor_c.mc3p_sync_data);
        uint8_t len;
        pwmSample_t *sample_ptr = pwmDataBuffer_sample(&pwmDataBuffer, &len);
        if(motor_c.mc3p.mode == ELDRIVER_MC3P_MODE_TRAP){
            //(*sample_ptr)[0] = (float)(motor_c.mc3p_sync_data.trap.vbus_q31);
            (*sample_ptr)[1] = (float)(motor_c.mc3p_sync_data.trap.vbemf_q31);
            //(*sample_ptr)[2] = (float)(motor_c.mc3p_sync_data.trap.cbus_q31);
        }
        pwmDataBuffer_pushSample(&pwmDataBuffer);
        
        switch (motor_c.state)
        {
        case ELMOTOR_IDLE:
            /* code */
            break;   
        case ELMOTOR_STUP_TRAPBEMF:
            /* code */
            pmsm_stup_trapbemf(&motor_c);
            break;
        case ELMOTOR_CL_TRAP:
            pmsm_cl_trap(&motor_c);
            break;

        default:
            break;
        }
        motor_c.xmc_ticks++;
    }
    eldriver_mc3p_write_trap(&motor_c.mc3p, motor_c.mc3p_sector, motor_c.mc3p_trap_duty_q15);
}


void eldriver_xmc3p_tickerCallback()
{

}




void pwmDataBuffer_init(pwmDataBuffer_t *cp)
{
    elcore_rstream_init(&cp->buffer, (void *)cp->frames, sizeof(PwmDataFrame_t), FRAME_BUFFER_COUNT);
    cp->frame_sample_idx = 0;
    cp->sample_count = 0;
    cp->overflowCount = 0;
    uint8_t *w2;
    uint16_t c1, c2;
    elcore_rstream_reserveWrite(&cp->buffer, 1, (void**)&cp->currentFrame, &c1, (void**)&w2, &c2);
    cp->currentFrame->sample_counter = 0;
    //update curernt frame value
}

pwmSample_t* pwmDataBuffer_sample(pwmDataBuffer_t *cp, uint8_t *sample_len)
{
    if(cp->frame_sample_idx < SAMPLES_PER_FRAME)
    {
        *sample_len = SAMPLE_LEN; //5 floats per sample
        return &cp->currentFrame->samples[cp->frame_sample_idx];
    }
    return NULL; // No available sample slot
}


void pwmDataBuffer_pushSample(pwmDataBuffer_t *cp)
{
    if(cp->frame_sample_idx < SAMPLES_PER_FRAME)
    {
        cp->frame_sample_idx++;
    }
    cp->sample_count++;
    if(cp->frame_sample_idx >= SAMPLES_PER_FRAME)
    {
        elcore_rstream_commitWrite(&cp->buffer, 1); // Commit the current frame
        //Current frame is full, move to next frame
        uint8_t *w2;
        uint16_t c1, c2;
        //Commit the current full frame
        if(elcore_rstream_reserveWrite(&cp->buffer, 1, (void**)&cp->currentFrame, &c1, (void**)&w2, &c2))
        {
            cp->currentFrame->sample_counter = cp->sample_count;
            cp->frame_sample_idx = 0;
        }else{
            //Buffer overflow, data loss occurs
            cp->overflowCount++;
        }
    }
}


bool pwmDataBuffer_readFrame(pwmDataBuffer_t *cp, PwmDataFrame_t **frame)
{
    uint8_t *r2;
    uint16_t c1, c2;
    if(elcore_rstream_peekRead(&cp->buffer, (void**)frame, &c1, (void**)&r2, &c2))
    {
        //Successfully read a frame
        //Process the frame data as needed
        //After processing, commit the read to move the tail forward
        elcore_rstream_releaseRead(&cp->buffer, 1);
        return true;
    }else{
        return false;
    }
}