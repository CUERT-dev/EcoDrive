#pragma once
#include "IController.h"


namespace pmsmControl{
    class TrapezoidalController : public IController{
        protected:

         enum class FeedbackType : bool{
            HallFeedback = 1,
            BemfFeedback = 0
        };

        friend class Controller;

        static void CommutateBEMF();
        static void CommutateHALL(uint8_t halVal);
        inline static uint8_t lastStep = 0;
        inline static uint8_t currentStep = 0;
        inline static int32_t stepsAccumulate = 0; //Keeps track of trapezoidal commutation steps
        inline static int32_t stepsAccumulateLast = 0; //The last amount of accumulated steps in the last update
        inline static uint8_t hallSequenceErrorCount = 0;
        inline static FeedbackType commutationFeedBackType = FeedbackType::HallFeedback; // true = HALL sensor feedback, false = BEMF feedback
        
        TrapezoidalController() = default;
        
        //Mosfet Driver functions and helper methods Optimized for Extreme speed and flexibility in implemenation

        static constexpr uint8_t hallToStep[]={
            /* 000 */ 0xFF, // Invalid
            /* 001 */ 0,
            /* 010 */ 2,
            /* 011 */ 1,
            /* 100 */ 4,
            /* 101 */ 5,
            /* 110 */ 3,
            /* 111 */ 0xFF  // Invalid
        };

        using TrapDriverWriteFn = void(*)(uint16_t);

        static constexpr TrapDriverWriteFn stepToDriverWrite[6]={
            &hBridgeDriver::Write_UQ16<PhaseState::H, PhaseState::Z, PhaseState::L>,
            &hBridgeDriver::Write_UQ16<PhaseState::H, PhaseState::L, PhaseState::Z>,
            &hBridgeDriver::Write_UQ16<PhaseState::Z, PhaseState::L, PhaseState::H>,
            &hBridgeDriver::Write_UQ16<PhaseState::L, PhaseState::Z, PhaseState::H>,
            &hBridgeDriver::Write_UQ16<PhaseState::L, PhaseState::H, PhaseState::Z>,
            &hBridgeDriver::Write_UQ16<PhaseState::Z, PhaseState::H, PhaseState::L>,
        };

        //Usage for forward commutation to get right phase states
        //stepToPhaseState[hallToStep[hall_val]]

        void Start();
        void Stop();

        public:

        bool Init() override;
        void Update(uint32_t t_ms) override;

        const void* GetStateAdv_R() override;
        ConfigFeedback SetConfig(Config &cfg) override;

        inline bool PrintConfig(char *buffer, uint8_t size, PrintMode mode);
        inline bool PrintStateAdv(char *buffer, uint8_t size, PrintMode mode);
    };

};



