#pragma once
#include "core/types.h"


namespace EcoDrive{
namespace MotorControl{
    
    class TrapezoidalController{

        public:

        struct Pwm_Config{
            types::polarity AH_polarity;
            types::polarity BH_polarity;
            types::polarity CH_polarity;

            types::polarity AL_polarity;
            types::polarity BL_polarity;
            types::polarity CL_polarity;

            float freq_hz;
            float deadtime_ns;
            float dutyMax_perc;
            float dutyMin_perc;
            
            bool breakEnabled;
        };


        struct HallSensor_Config{
            types::polarity A_polarity;
            types::polarity B_polarity;
            types::polarity C_polarity;

            float commutationDelay_rad;
            bool exist = false;
            int8_t forwardDir = 1;
        };

        struct VoltageSensor_Config{
            voltageSensorType type;

            float maximumBusVoltage_volt;
            float minimumBusVoltage_volt;
        };

        struct CurrentSensor_Config{
            currentSensorType type;

            float maximumPhaseCurrent_amp;
            float minimumPhaseCurrent_amp;
        };

        struct Config{
            controlMode mode;
            positionSensorType postionSensor;
            uint8_t pwmFreq_to_controlLoopFreq_ratio;
        };

        struct motorParameter{
            float polePairs;
            float fluxLinkage_Wb;
            float statorResistance_ohm;
            float statorInductance_H;
            float inertia_kgm2;
            float friction_NmPs;
            float KV_rpmPerVolt;

            float maximumCurrent;
            float maximumVoltage;
        };


        void init(const Config cfg,
                  const Pwm_Config &pwm_cfg,
                  const HallSensor_Config &hall_cfg,
                  const VoltageSensor_Config &voltage_cfg,
                  const CurrentSensor_Config &current_cfg);

        
        
        ~TrapezoidalController() = default;


    }

}
}