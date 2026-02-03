#pragma once

#include "platform/driver_core/hall1.h"
#include "driver/driver_common.h"
#include "core/math.hpp"

template<uint8_t instance>
class BemfZc{
    private:
    uint32_t last_TRAP_sector = 0;
    bool last_bemf_state = 0; //Tracks the crossing direction for hysteresis functionality
    bool isZeroCrossedNow = 0;
    int32_t triggerCount = 0;
    uint32_t periodicTime_uS = 0;

    Hall_Config config;

    public:
    
    void init(const Hall_Config cfg)
    {
        config = cfg;
        if constexpr(instance == 1)hall1_init();
    }

    void update(const uint32_t t_nS, const float u_V, const float v_V, const float w_V, const uint8_t TRAP_sector)
    {
        isZeroCrossedNow = false;
        float vn_V = (u_V + v_V + w_V)/3;
        float bemf_V = 0;
        // Handle 6→1 wrap-around and 1→6 reverse wrap
        int8_t direction = TRAP_sector - last_TRAP_sector;
        
        // Handle wrap-around cases (we dont expect any more than one differences , yet even up to 3 difference still works)
        if (direction == 5 || direction == -5) {
            direction = (direction == 5) ? -1 : 1;  // 1→6 = -1, 6→1 = +1
        }

        // Here detect which one is the floating phase based on trapezoidal commutation sector
        // Back emf = floating phase measured voltage - neutral voltage (all with respect to ground)
        switch (TRAP_sector)
        {
            case 1:case 4: bemf_V = w_V - vn_V; break;
            case 2:case 5: bemf_V = v_V - vn_V; break;
            case 3:case 6: bemf_V = u_V - vn_V; break;
            default:
                break;
        }

        //Zero crossing hysteresis functionality
        if(last_bemf_state == true && bemf_V < -config.hysteresisVoltage_V)
        {
            last_bemf_state = false;
            float ElecVelocity = getElecVelocity_DegpS();
            ElecVelocity = ElecVelocity < 0? -ElecVelocity : ElecVelocity;
            if(ElecVelocity > config.minimumElecVelocity_DegpS){
                uint32_t delay_uS = UNIT_TO_MICRO(config.commutationDelay_DegElec/ElecVelocity);
                if constexpr(instance == 1){
                isZeroCrossedNow = true;
                hall1_setCOMDelay(delay_uS);
                hall1_triggerSWChange();
                }
            }
        }
        else if(last_bemf_state == false && bemf_V > config.hysteresisVoltage_V)
        {
            last_bemf_state = true;
            float ElecVelocity = getElecVelocity_DegpS();
            ElecVelocity = ElecVelocity < 0? -ElecVelocity : ElecVelocity;
            if(ElecVelocity > config.minimumElecVelocity_DegpS){
                uint32_t delay_uS = UNIT_TO_MICRO(config.commutationDelay_DegElec/ElecVelocity);
                if constexpr(instance == 1){
                isZeroCrossedNow = true;
                hall1_setCOMDelay(delay_uS);
                hall1_triggerSWChange();
                }
            }
        }
        //Else Either in between value with no trigger or not near zero
        
        triggerCount += direction;
        last_TRAP_sector = TRAP_sector;
    }
    
    inline int32_t getElecAngle_Deg()
    {
        return (triggerCount%6)*60;
    }

    void reset(float angle_Deg = 0)
    {
        triggerCount += angle_Deg/60;
    }

    bool zeroCrossedNow()
    {
        return isZeroCrossedNow;
    }
    float getElecVelocity_DegpS()
    {
        uint16_t period_uS;
        if constexpr(instance == 1)period_uS = hall1_getPeriod_uS();
        if(period_uS == 0) return 0x7FFFFFFF;//oveflow
        //Some other calculations here
        return UNIT_TO_MICRO(60.0)/period_uS;
    }

    inline void enableCOMCallback(void(*cb)())
    {
        if constexpr(instance == 1)return hall1_setCOMEVENTCallback(cb); 
    }

    inline void stopCOMCallback()
    {
        if constexpr(instance == 1)return hall1_setCOMEVENTCallback(nullptr);
    }
};


