#pragma once
#include "driver/driver_common.h"
#include "platform/driver_core/MCADCPWM3P.h"


template<uint16_t instance>
struct MCADCPWM3P{
    constexpr static inline void init(const MCADCPWM3P_Config &cfg)
    {
        if constexpr (instance == 1)return MCADCPWM3P_init(cfg);
    }

    constexpr static inline int32_t adcReg_read()
    {
        if constexpr (instance == 1)return MCADCPWM3P_adcReg_read();
    }

    constexpr static inline void adcPwm_read(int32_t scanData[6])
    {
        if constexpr (instance == 1)return MCADCPWM3P_adcPwm_read(scanData);
    }

    constexpr static void pwmSVM_write(int16_t dutyU_q15, int16_t dutyV_q15, int16_t dutyW_q15)
    {
        if constexpr (instance == 1)MCADCPWM3P_pwmSVM_write(dutyU_q15, dutyV_q15, dutyW_q15);
    }

    constexpr static void pwmTRAP_write(int8_t sector, uint16_t duty_q15)
    {
        if constexpr (instance == 1)MCADCPWM3P_pwmTRAP_write(sector, duty_q15);
    }

};



