#include "pmsmControl/pmsmControl.h"
#include <stdio.h>
#include "platform/platform.h"
#include "driver/ISerial.h"
#include "FreeRTOS.h"
#include "middleware/aebfv0.h"

Uart_Config ucfg = {115200, Uart_Parity::NONE, Uart_StopBit::ONE};
const char buffer[] = "HELLO_RECEIVER_ASS";
uint8_t output_buffer[100];
uint8_t ass_buffer[100];
char s = '\0';
PmsmControl pmsmController1;
int main(){
    platform_init();
    Serial<USBVCOM1>::init(ucfg);
    pmsmController1.init();
    while (true) {
        char fuck[] = {0xFF, 0xFF, 0xFF, 0xFF};
        aebp_encode_frame(output_buffer,
                        4, 6, 
                        (uint8_t*)buffer, sizeof(buffer));
        Serial<USBVCOM1>::write(output_buffer,  AEBP_FRAMED_SIZE(sizeof(buffer)));
        HAL_Delay(1000);
        Serial<USBVCOM1>::write((uint8_t*)fuck, 4);
        HAL_Delay(1000);

        uint8_t payloadLen;
        uint8_t dv;
        uint16_t sv;
        
        volatile uint8_t status = aebp_decode_frame(output_buffer, AEBP_FRAMED_SIZE(sizeof(buffer)),
                        &dv, &sv, 
                        ass_buffer, &payloadLen);
        Serial<USBVCOM1>::write(ass_buffer,  payloadLen);
        HAL_Delay(1000);
    }

    return 0;
}