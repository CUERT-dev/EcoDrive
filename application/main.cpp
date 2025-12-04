#include "pmsmControl.hpp"
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "platforms/stm32f4xx/platform.h"
#include "drivers/communication/IUart.hpp"




PmsmControl pmsmController1;

int main(){
    platform_init();
    DPIO_init();
    
    pmsmController1.init();

    while (true) {

    }

    return 0;
    
}