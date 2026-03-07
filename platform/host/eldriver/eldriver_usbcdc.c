#include "eldriver_usbcdc.h"
#include <stdio.h>
#ifdef ELDRIVER_USBCDC_ENABLED

void eldriver_usbcdc_init(eldriver_uart_handle_t *handle)
{

}

uint8_t eldriver_usbcdc_write(eldriver_uart_handle_t *handle, uint8_t* data, uint8_t len)
{    
    printf("SAD");
}

uint16_t eldriver_usbcdc_read(eldriver_uart_handle_t *handle, uint8_t* data, uint8_t len)
{

}

elcore_rstream_stats_t eldriver_usbcdc_rx_stats(eldriver_uart_handle_t *handle)
{
   
}

elcore_rstream_stats_t eldriver_usbcdc_tx_stats(eldriver_uart_handle_t *handle)
{
    
}

void eldriver_usbcdc_resetStats(eldriver_uart_handle_t *handle)
{

}

#endif