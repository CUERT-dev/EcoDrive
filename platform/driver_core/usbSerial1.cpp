#include "usbSerial1.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

void usbSerial1_init(const Uart_Config &cfg)
{
    MX_USB_DEVICE_Init();
}

bool usbSerial1_write(const uint8_t* data, uint8_t len)
{    
    return (CDC_Transmit_FS((uint8_t*)data, len) == USBD_OK);
}

uint16_t usbSerial1_read(uint8_t* data, uint8_t len)
{
    return 0;
}

uint8_t usbSerial1_available()
{
    return 0;
}
