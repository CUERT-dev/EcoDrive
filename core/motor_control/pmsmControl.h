#pragma once

#include "IController.h"
#include "TrapezoidalController.h"


namespace pmsmControl{

enum class ControllerType: uint8_t{
    Trapezoidal = 0,
    Modulation = 1,
    Foc = 2    
};



class Controller{
    private:
    static inline ControllerType activeController = ControllerType::Trapezoidal;
    static inline TrapezoidalController trapezoidalController;
    Controller() = delete;

    public:

    static IController* SetActiveController(ControllerType type)
    {
        activeController = type;
        switch (type)
        {
        case ControllerType::Trapezoidal:
            return &trapezoidalController;
            break;
        
        case ControllerType::Modulation:
            //Return instance for a Modulation controller
        case ControllerType::Foc:
            //Return and instance for a Foc Controller
            return nullptr;
        default:
            return nullptr;
            break;
        }
        return nullptr;
    }   

    static IController* GetActiveController(){
        return SetActiveController(activeController);
    }

    static ControllerType GetActiveControllerType(){
        return activeController;
    }
    
};

};