#pragma once

#include "Config.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "Hall/Hall.hpp"
#include "HBridge/HBridge.hpp"

namespace pmsmControl{
    
    enum class Direction : bool{
        Forward = 1,    
        Backward = 0
    };

    template <typename T>
    struct FlagPack{
        T flagsPack;
        inline void set(uint8_t idx){flagsPack |= (1<<(idx) );};
        inline void clear(uint8_t idx){flagsPack &= ~(1<<(idx) );};
        inline bool read(uint8_t idx){return flagsPack & (1<<(idx) );}
    };

    enum class PrintMode{
        CBOR_BIN,
        TXT
    };
    

    using hall = HallSensor<0>;
    using hBridgeDriver = HBridgeDriver<0>;


    class IController{ 
        public:
        
        struct Config{
            // Add your config methods/data here
            protected:
                friend class TrapezoidalController;
                friend class IController;
                uint16_t effDuty_UQ16;

            public:
                float effDuty_float;
                Direction direction;
                uint8_t polePairs;
                uint16_t switching_freq;
                float MaxDuty;
                float MinDuty;

            Config(){
                effDuty_float = 0;
                direction = Direction::Forward;
                polePairs = 2;
                switching_freq = 10000;
                MaxDuty = 0.95;
                MinDuty = 0.05;
            }
        };

        struct ConfigFeedback : public FlagPack<uint8_t>{
            enum Bit: uint8_t{
                isOk = 0,
                isOverFreq = 1,
                isUnderFreq = 2,
                isOverDuty = 3,
                isUnderDuty = 4,
            };

            ConfigFeedback(){
                set(isOk);
                clear(isOverFreq);
                clear(isOverDuty);
                clear(isUnderFreq);
                clear(isUnderDuty);
            }
        };

        struct State{
            struct stateFlags : public FlagPack<uint8_t>{
                enum Bit: uint8_t{
                    hardFault = 0,
                    running = 1,
                    startFail = 2,
                    hallConnected = 3,
                    overCurrent = 4,
                };
            }flags;
            float angle_rad;
            float angularVelocity_radPERs;

            State(){
                flags.clear(stateFlags::hardFault);
                flags.clear(stateFlags::running);
                flags.clear(stateFlags::startFail);
                flags.set(stateFlags::hallConnected);
                flags.clear(stateFlags::overCurrent);
                angle_rad = 0;
                angularVelocity_radPERs = 0;
            };
        };

        virtual ~IController() = default;
        virtual bool Init();
        virtual void Update(uint32_t t_ms);

        const State& GetState(){return state;};
        virtual const void* GetStateAdv_R();

        const Config& GetConfig(){return config;}
        virtual ConfigFeedback SetConfig(Config& cfg);

        bool PrintState(char* buffer, uint8_t size, PrintMode mode);
        bool PrintConfig(char* buffer, uint8_t size, PrintMode mode);
        virtual bool PrintStateAdv(char* buffer, uint8_t size, PrintMode mode);
    
    protected:
        constexpr uint16_t float_to_UQ16(float val){return static_cast<uint16_t>(val * 0xFFFF);};
        constexpr float UQ16_to_float(uint16_t val){return static_cast<float>(float(val) / 0xFFFF);}

        inline static Config config = Config();
        inline static State state = State();
        uint32_t updateTimeLast_ms = 0;    
        //Messages
        static constexpr const char* PrintState_txt = "<State:{F=%X A=%" PRId32 " AV=%" PRId32 "}>";
        static constexpr const char* PrintConfig_txt = "<Config:{D=%" PRIu16 " DIR=%c P=%" PRIu8 " SWF=%" PRIu16 "}>"; 
        // Protected constructor ensures only derived classes can instantiate
        IController() = default;
        /********************End of Interface structure IController (Any other controller variant for pmsm should inherit from it) *******************/
    };

    /*************END OF pmsmControl Namespace ******************************/
};

