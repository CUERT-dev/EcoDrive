#pragma once
#include "core/motorControl/types.h"

namespace EcoDrive{



class PWM{
    public:

    struct Config{
        Types::Polarity AH_polarity;
        Types::Polarity BH_polarity;
        Types::Polarity CH_polarity;
        Types::Polarity AL_polarity;
        Types::Polarity BL_polarity;
        Types::Polarity CL_polarity;
        float freq_hz;
        float deadtime_ns;
        float dutyMax_perc;
        float dutyMin_perc;
        
        Types::PWMAlignmentType pwmAlignment; 
        Types::PWMModulationType modulation;
        bool breakEnabled;
    };


    void init(const Config cfg);
    void write(float dutyA_perc, float dutyB_perc, float dutyC_perc);
    bool isHealthy();
    void breakFcn();
    ~PWM() = default;
};

};
