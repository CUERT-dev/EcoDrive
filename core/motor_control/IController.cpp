
#include "IController.h"
#include "TrapezoidalController.h"

using namespace pmsmControl;


bool IController::PrintConfig(char* buffer, uint8_t size, PrintMode mode){
    if(mode == PrintMode::TXT){
        int8_t writeNum = snprintf(buffer, size, PrintConfig_txt,
            (uint16_t)(config.effDuty_float*100), 
            (config.direction==Direction::Forward? 'F': 'B'),
            config.polePairs,
            config.switching_freq
        );
        return (writeNum>size || writeNum<0) ? false:true;
    }else if(mode == PrintMode::CBOR_BIN){
        return false;
    }

    //This line Shouldn't be reached if things go as designed
    return false;
}

bool IController::PrintState(char* buffer, uint8_t size, PrintMode mode){ 
    if(mode == PrintMode::TXT){
        int8_t writeNum = snprintf(buffer, size, PrintState_txt, 
            state.flags.flagsPack,
            (int32_t)(state.angle_rad*1000),
            (int32_t)(state.angularVelocity_radPERs*1000)
        );
        return (writeNum>size || writeNum<0) ? false:true;
    }else if(mode == PrintMode::CBOR_BIN){
        return false;
    }

    //This line Shouldn't be reached if things go as designed
    return false;
}

