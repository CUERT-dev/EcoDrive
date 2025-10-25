#pragma once
#include "core/motorControl/types.h"


namespace EcoDrive{
namespace PmsmControl{
    

class TrapezoidalController{
    public:
    
    struct Config{
        int8_t forwardDir = 1;
    };


    void init(const Config cfg);
    
    ~TrapezoidalController() = default;
};

};
};