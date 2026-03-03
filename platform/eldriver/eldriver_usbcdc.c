#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "eldriver_usbcdc.h"

#ifdef ELDRIVER_USBCDC_ENABLED
#pragma message "FUCK YOU"
void eldriver_usbcdc_init(eldriver_uart_handle_t *handle)
{
    elcore_rstream_init(&cdc_tx_buffer, UserTxBufferFS, 1, APP_TX_DATA_SIZE);
    elcore_rstream_init(&cdc_rx_buffer, UserRxBufferFS, 1, APP_RX_DATA_SIZE);
    
    MX_USB_DEVICE_Init();
}

uint8_t eldriver_usbcdc_write(eldriver_uart_handle_t *handle, uint8_t* data, uint8_t len)
{    
    return CDC_Transmit_FS((uint8_t*)data, len);
}

uint16_t eldriver_usbcdc_read(eldriver_uart_handle_t *handle, uint8_t* data, uint8_t len)
{
    return 0;
}

elcore_rstream_stats_t eldriver_usbcdc_rx_stats(eldriver_uart_handle_t *handle)
{
    return elcore_rstream_getStats(&cdc_rx_buffer);
}

elcore_rstream_stats_t eldriver_usbcdc_tx_stats(eldriver_uart_handle_t *handle)
{
    return elcore_rstream_getStats(&cdc_tx_buffer);
}

void eldriver_usbcdc_resetStats(eldriver_uart_handle_t *handle)
{
    elcore_rstream_resetStats(&cdc_rx_buffer);
    elcore_rstream_resetStats(&cdc_tx_buffer);
}

#endif