#include "TrapezoidalController.h"



using namespace pmsmControl;

bool TrapezoidalController::Init(){
    bool success = true;
    hBridgeDriver::Config hBridgeDriverConfig{
        .pwmFreq = 10000,
    };


    hBridgeDriver::Init(hBridgeDriverConfig);
    hall::AttachHallChangeCallBack(&TrapezoidalController::CommutateHALL);
    hall::Init();
    return success;
}

IController::ConfigFeedback TrapezoidalController::SetConfig(Config &cfg)
{
    ConfigFeedback feedback;
    if(cfg.effDuty_float > maxSpeed_duty)cfg.effDuty_float = maxSpeed_duty;
    if(cfg.effDuty_float < minSpeed_duty)cfg.effDuty_float = 0;

    hBridgeDriver::Config hBridgeDriverConfig{
        .pwmFreq = cfg.switching_freq,
    };

    hBridgeDriver::SetConfig(hBridgeDriverConfig);

    config = cfg;
    return feedback;
}


const void* TrapezoidalController::GetStateAdv_R()
{
    return nullptr;
}

inline bool TrapezoidalController::PrintConfig(char *buffer, uint8_t size, PrintMode mode)
{
    return false;
}

inline bool TrapezoidalController::PrintStateAdv(char *buffer, uint8_t size, PrintMode mode)
{
    return false;
}

void TrapezoidalController::Start()
{
    //Check for hall Sensor Connection First
    const uint8_t hallNum = hall::Read();
    if(hallNum == 0xFF){
        state.flags.clear(State::stateFlags::hallConnected);
        state.flags.set(State::stateFlags::startFail);
    }
    if(!state.flags.read(State::stateFlags::hardFault) && !state.flags.read(State::stateFlags::running)){
        if(commutationFeedBackType == FeedbackType::HallFeedback && state.flags.read(State::stateFlags::hallConnected)){
            CommutateHALL(hall::Read());
            state.flags.set(State::stateFlags::running);
        }else{
            CommutateBEMF();
        }
    }
}

void TrapezoidalController::Stop(){
    //Disable All phases (ie. leave them floating)
    hBridgeDriver::Write_UQ16<PhaseState::Z, PhaseState::Z, PhaseState::Z>(0);
    state.flags.clear(State::stateFlags::running);
}


void TrapezoidalController::CommutateHALL(uint8_t hallVal)
{
    // 1. Read hall sensors (single atomic read if possible)
    const uint8_t hallNum = hallVal;
    currentStep = hallToStep[hallNum];
    if(currentStep == lastStep)return;
    // 2. Detect disconnection or some faults due to bit flipping
    if(currentStep == 0xFF){
        state.flags.clear(State::stateFlags::hallConnected);
        hallSequenceErrorCount++;
        return;
    }

    // 3. Calculate step difference 
    int8_t delta = ((currentStep - lastStep + 6 + 3) % 6) - 3;   //Calculate step difference 
    
    // 4. If delta isn't in the bound of -1 , 1 we missed a step :< or sequence isn't proper
    if(delta > 1 || delta < -1){
        hallSequenceErrorCount++;
    }

    /////If we reach here no faults happened related to hall::effect sensor

    // 5. calculate the number of hall::triggers and the direction  +Ve Forward and -Ve for Backward
    stepsAccumulate += delta;

    // 6. drive the 3phase H-bridge according to commutation table accordingly 
    if(state.flags.read(State::stateFlags::running) && commutationFeedBackType == FeedbackType::HallFeedback){
        if(config.direction == Direction::Forward){
            stepToDriverWrite[currentStep] (config.effDuty_UQ16 );
        }else{
            stepToDriverWrite[5 - currentStep]( config.effDuty_UQ16 );
        }
    }
    
    lastStep = currentStep;
}

void TrapezoidalController::CommutateBEMF(void){
    //Yet to be implemented
}

constexpr float PI = 3.14159265359f;
constexpr uint8_t ELECTRICAL_SEGMENTS_PER_REV = 6; // 6 steps per electrical rev

void TrapezoidalController::Update(uint32_t t_ms)
{
    // Calculate delta triggers since last update
    int32_t delta_triggers = stepsAccumulate - stepsAccumulateLast;
    
    // Update rotations (total mechanical revolutions)
    state.angle_rad += static_cast<float>(  stepsAccumulate /
               (ELECTRICAL_SEGMENTS_PER_REV * config.polePairs) );
    
    uint32_t delta_t_ms = t_ms - updateTimeLast_ms;
    // Calculate angular velocity (rad/s)
    if (delta_t_ms > 0) {
        float electricalRevs = static_cast<float>(delta_triggers) / ELECTRICAL_SEGMENTS_PER_REV;
        float mechanicalRevsPerSec = (electricalRevs / config.polePairs / delta_t_ms)/1000;
        state.angularVelocity_radPERs = mechanicalRevsPerSec * 2.0f * PI;
    }
    
    // Update last value
    updateTimeLast_ms = t_ms;
    stepsAccumulateLast = stepsAccumulate;
}
