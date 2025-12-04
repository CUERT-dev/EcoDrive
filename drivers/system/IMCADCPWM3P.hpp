#pragma once
#include "drivers/common.h"
#include "drivers/stm32f4/MCADCPWM3P.h"


template<uint16_t instance>
struct MCADCPWM3P{
    constexpr static inline void init(const MCADCPWM3P_Config &cfg)
    {
        if constexpr (instance == 1)return MCADCPWM3P_init(cfg);
    }

    constexpr static inline uint16_t adcReg_read(uint8_t ch)
    {
        if constexpr (instance == 1)return MCADCPWM3P_adcReg_read(ch);
    }

    constexpr static inline void adcPwm_registerPostScanCallback(void (*callback)())
    {
        if constexpr (instance == 1)return MCADCPWM3P_adcPwm_registerPostScanCallback(callback);
    }

    constexpr static inline void adcPwm_read(int32_t scanData[6])
    {
        if constexpr (instance == 1)return MCADCPWM3P_adcPwm_read(scanData);
    }

    constexpr static void pwm_write(int16_t dutyU_q15, int16_t dutyV_q15, int16_t dutyW_q15)
    {
        if constexpr (instance == 1)MCADCPWM3P_pwm_write(dutyU_q15, dutyV_q15, dutyW_q15);
    }

};



