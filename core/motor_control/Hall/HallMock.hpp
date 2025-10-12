#pragma once
#include "IHall.hpp"

template<uint8_t Instance>
class HallMock : public IHall<HallMock<Instance>> {
private:
    friend class IHall<HallMock<Instance>>;
    static inline uint8_t s_mockState = 0;
    static bool InitImpl() { return true; }

    static uint8_t ReadImpl() { 
        return s_mockState; 
    }
public:
    static void SetState(uint8_t state) { 
        s_mockState = state & 0x07; 
        IHall<HallMock>::triggerCallback(); // Simulate Change interrupt
    }
};