#pragma once
#include <stdint.h>


//Hall state bit field ABC order
//Static requirement for Interrupts
// CRTP design pattern for Interface class
template<typename Derived>
class IHall{
    protected:
    static inline void(*cbMethod)(uint8_t) = nullptr;
    static inline uint8_t readABC = 0;

    static inline void TriggerCallback(){
        readABC = Read();
        if(cbMethod){
            cbMethod(readABC);
        }
    };

    IHall() = default;
    ~IHall() = default;

    public:
    static bool Init(){
        return Derived::InitImpl();
    };
    static uint8_t Read(){
        return Derived::ReadImpl();
    };
    static void AttachHallChangeCallBack(void(*method)(uint8_t)){
        cbMethod = method;
    }
};
/************************End Of Hall static structure (Static requirements enforced for speed and ISR registering requirements) *****************/






