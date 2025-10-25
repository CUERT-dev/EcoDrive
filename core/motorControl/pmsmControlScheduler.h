#pragma once


#include "core/types.h"

namespace EcoDrive{

    class pmsmControlScheduler{
    public:
        void init();
        void update(Types::pwmTicks_t currentPwmTick);
        ~pmsmControlScheduler() = default;
    }
}
