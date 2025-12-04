#include "drivers/communication/IUart.hpp"
#include "core/ringbuffer.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "driver_conf.h"
#include "driver_bsp.h"
#include "uart1.h"
#include <string.h>

#ifdef DRIVER_USE_UART_I1

//======================================================
//DMA static definitions for UART1 DMA and UART instance
//======================================================
constexpr uint8_t UART1_TX_DMA_BUFFER_SIZE = 32;
constexpr uint8_t UART1_RX_DMA_BUFFER_SIZE = 64;

static char uart1_dma_uart1_tx_buffer[UART1_TX_DMA_BUFFER_SIZE];
static char uart1_dma_rx_buffer[UART1_RX_DMA_BUFFER_SIZE];
#define UART1_INSTANCE USART1

//NEVER FORGET 
//THAT YOU EVER CHANGE THESE VALUES FOR DMA YOU STILL HAVE TO CHANGE IMPLEMENTATION AND IRQ APPROPIATELY
#define UART1_DMA_TX_STREAM LL_DMA_STREAM_7
#define UART1_DMA_RX_STREAM LL_DMA_STREAM_5
#define UART1_DMA_CHANNEL LL_DMA_CHANNEL_4
#define UART1_DMA_INSTANCE DMA2

void uart1_dma_transmit(size_t len);
void uart1_dma_receive_handle();

static void uart1_dma_transmit_complete_callback();

#if UART_I1_DMA_RECEIVE_COMPLETE_CALLBACK_ENABLED
void (*uart1_dma_receive_complete_callback)(void) = nullptr;
#endif 

void uart1_dma_receive_complete_register_callback(void(*func)(void));




void uart1_dmaInit(const Uart_Config &cfg);
void uart1_interruptInit(const Uart_Config &cfg);
void uart1_uartInit(const Uart_Config &cfg);
void uart1_init(const Uart_Config &cfg);
const Uart_Status uart1_getStatus();
void uart1_clearErrors();
void uart1_errorCallback();
bool uart1_write(char* data , uint8_t len);
uint16_t uart1_read(char *data, uint8_t len);


//================================================
// Static memory management definitions for USART1
//================================================
constexpr uint32_t UART1_TX_DRV_BUFFER_SIZE = 256;
constexpr uint32_t UART1_RX_DRV_BUFFER_SIZE = 256;

// Rx and Tx Buffer management
char uart1_rx_buf[UART1_RX_DRV_BUFFER_SIZE];
char uart1_tx_buf[UART1_TX_DRV_BUFFER_SIZE];
ring_buffer_t uart1_rx_ringBuffer;
ring_buffer_t uart1_tx_ringBuffer;

//==============================================
// Static member definitions for USART1
//==============================================
Uart_Instance uart1;

//============================================
// HARDWARE INITIALIZATION FUNCTIONS
//============================================
void uart1_dmaInit(const Uart_Config &cfg)
{

    // Enable DMA2 clock
    __HAL_RCC_DMA2_CLK_ENABLE();

    // Enable DMA clock
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    
    //#######
    // RX DMA Configuration (Circular)
    //#######
    LL_DMA_SetChannelSelection(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, UART1_DMA_CHANNEL);
    LL_DMA_SetDataTransferDirection(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM);
    
    LL_DMA_SetPeriphAddress(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, (uint32_t)&USART1->DR);
    
    // Enable RX DMA interrupts
    LL_DMA_EnableIT_TC(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM);
    LL_DMA_EnableIT_HT(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM);

    LL_DMA_SetMemoryAddress(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, (uint32_t)uart1_dma_rx_buffer);
    
    //Start the DMA reception on Rx!
    LL_DMA_SetDataLength(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, UART1_RX_DMA_BUFFER_SIZE);
    LL_DMA_EnableStream(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM);
    
    //#######
    // TX DMA Configuration (Normal)
    //#######
    LL_DMA_SetChannelSelection(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, UART1_DMA_CHANNEL);
    LL_DMA_SetDataTransferDirection(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM);
    
    LL_DMA_SetPeriphAddress(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, (uint32_t)&USART1->DR);
    
    // Enable TX DMA interrupt
    LL_DMA_EnableIT_TC(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM);
}

void uart1_interruptInit(const Uart_Config &cfg)
{
    // DMA interrupts
    NVIC_SetPriority(DMA2_Stream5_IRQn, 1);
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    
    NVIC_SetPriority(DMA2_Stream7_IRQn, 1);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    
    // UART interrupt
    NVIC_SetPriority(USART1_IRQn, 0);
    NVIC_EnableIRQ(USART1_IRQn);
    
}


void uart1_uartInit(const Uart_Config &cfg)
{ 
    // Enable USART1 clock (matching GPIO pins)
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    // Configure UART
    LL_USART_SetBaudRate(USART1, SystemCoreClock, LL_USART_OVERSAMPLING_16, cfg.baud);
    LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
    
    // Stop bits
    LL_USART_SetStopBitsLength(USART1, 
        (cfg.stopBit == Uart_StopBit::ONE) ? LL_USART_STOPBITS_1 : LL_USART_STOPBITS_2);
    
    // Parity
    switch(cfg.parity) 
    {
        case Uart_Parity::NONE: LL_USART_SetParity(USART1, LL_USART_PARITY_NONE); break;
        case Uart_Parity::EVEN: LL_USART_SetParity(USART1, LL_USART_PARITY_EVEN); break;
        case Uart_Parity::ODD:  LL_USART_SetParity(USART1, LL_USART_PARITY_ODD); break;
    }
    
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
    LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
    
    // Enable DMA for RX and TX
    LL_USART_EnableDMAReq_RX(USART1);
    LL_USART_EnableDMAReq_TX(USART1);
    
    // Enable IDLE interrupt
    LL_USART_EnableIT_IDLE(USART1);

    LL_USART_Enable(USART1);
}

//===========================================
// FULL INITIALIZATION FUNCTION
//============================================
void uart1_init(const Uart_Config &cfg)
{
    // Initialize GPIO first
    uart1_gpioInit();
    uart1_dmaInit(cfg);
    uart1_interruptInit(cfg);
    uart1_uartInit(cfg);

    // Initialize buffer management variables
    ring_buffer_init(&uart1_rx_ringBuffer, uart1_rx_buf, UART1_RX_DRV_BUFFER_SIZE);
    ring_buffer_init(&uart1_tx_ringBuffer, uart1_tx_buf, UART1_TX_DRV_BUFFER_SIZE);
}

//===========================================
// UTILITY FUNCTIONS
//============================================
const Uart_Status uart1_getStatus()
{
    return uart1.status;
}

void uart1_clearErrors()
{
    
}

void uart1_errorCallback()
{
    // Handle errors here
    uart1_clearErrors();
}

void uart1_dma_transmit(size_t len)
{    
    LL_DMA_DisableStream(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM);
    LL_DMA_SetMemoryAddress(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, (uint32_t)uart1_dma_uart1_tx_buffer);
    LL_DMA_SetDataLength(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, len);
    LL_DMA_EnableStream(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM);
}

void uart1_dma_receive_handle()
{
    static uint16_t uart1_dma_lastPos = 0;
    uint16_t currentPos = UART1_RX_DMA_BUFFER_SIZE - LL_DMA_GetDataLength(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM);
    uint16_t bytes_received = 0;
    
    // Calculate bytes received since last call
    if (currentPos >= uart1_dma_lastPos) {
        // Normal case: no wrap-around
        bytes_received = currentPos - uart1_dma_lastPos;
    } else {
        // WRAP-AROUND case: TC occurred
        bytes_received = (UART1_RX_DMA_BUFFER_SIZE - uart1_dma_lastPos) + currentPos;
    }
    
    if(bytes_received > 0)
    {
        // Handle bulk copy based on wrap-around scenario
        if (currentPos >= uart1_dma_lastPos) {
            // Single contiguous block
            ring_buffer_queue(&uart1_rx_ringBuffer, 
                             uart1_dma_rx_buffer + uart1_dma_lastPos, 
                             bytes_received);
        } else 
        {
            // Two blocks: from lastPos to end, then from start to currentPos
            uint16_t first_chunk = UART1_RX_DMA_BUFFER_SIZE - uart1_dma_lastPos;
            uint16_t second_chunk = currentPos;
            
            // Copy first chunk (end of buffer)
            ring_buffer_queue(&uart1_rx_ringBuffer, 
                             uart1_dma_rx_buffer + uart1_dma_lastPos, 
                             first_chunk);
            
            // Copy second chunk (start of buffer) if any
            if (second_chunk > 0) {
                ring_buffer_queue(&uart1_rx_ringBuffer, 
                                 uart1_dma_rx_buffer, 
                                 second_chunk);
            }
        }
        
        uart1_dma_lastPos = currentPos;
    }
}

void uart1_dma_transmit_complete_callback()
{
    //Check if Ring Buffer has data to transmit and start dma transmission else do nothing
    if(!ring_buffer_is_empty(&uart1_tx_ringBuffer))
    {
        uint8_t len = ring_buffer_num_items(&uart1_tx_ringBuffer);
        if(len >= UART1_TX_DMA_BUFFER_SIZE)
        {
            len = UART1_TX_DMA_BUFFER_SIZE;
        }
        ring_buffer_dequeue(&uart1_tx_ringBuffer, uart1_dma_uart1_tx_buffer, len);
        uart1_dma_transmit(len);
    }
}

#if DMA_RECEIVE_CALLBACK_ENABLED
void uart1_dma_receive_complete_register_callback(void(*func)(void))
{
    uart1_dma_receive_complete_callback = func;
}
#endif

//===========================================
// READ AND WRITE FUNCTION DEFINTIONS
//============================================
bool uart1_write(const char* data , uint8_t len)
{
    //if The Ring Buffer For TX is empty and if DMA is not bust and load a chunk to dma directly else load to driver ringBuffer
    if(ring_buffer_is_empty(&uart1_tx_ringBuffer) && !LL_DMA_GetDataLength(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM))
    {
        size_t copy_len = (len > UART1_TX_DMA_BUFFER_SIZE) ? UART1_TX_DMA_BUFFER_SIZE : len;
        //copy to dma buffer
        memcpy(uart1_dma_uart1_tx_buffer, data, copy_len);
        //start dma transmission
        uart1_dma_transmit(len);
        //push remaining data to ring buffer if data was larger than dma buffer
        if(len > UART1_TX_DMA_BUFFER_SIZE)
        {
            ring_buffer_queue(&uart1_tx_ringBuffer, data + copy_len, len - copy_len);
        }
    }
    else //TX ring buffer already has data and DMA is transmiting push to ring buffer
    {
        ring_buffer_queue(&uart1_tx_ringBuffer, data, len);
    }
    return true;
}

uint16_t uart1_read(char* data, uint8_t len)
{   
    if(len > 0)
    {
        return ring_buffer_dequeue(&uart1_rx_ringBuffer, data, len);
    }
    return 0;
}

uint8_t uart1_available()
{
    return ring_buffer_num_items(&uart1_rx_ringBuffer);
}

//===========================================
// CALLBACK AND INTERRUPT DEFINTIONS
//============================================
// External C callbacks for IRQ Handlers
extern "C" {

    
    void DMA2_Stream5_IRQHandler(void) {
        // RX DMA Interrupt
        if (LL_DMA_IsActiveFlag_TC5(DMA2)) 
        {
            LL_DMA_ClearFlag_TC5(DMA2);
            // Handle transfer complete if needed
            uart1_dma_receive_handle();
        }
        if (LL_DMA_IsActiveFlag_HT5(DMA2)) 
        {
            LL_DMA_ClearFlag_HT5(DMA2);
            // Handle half transfer if needed
            uart1_dma_receive_handle();
        }
        if (LL_DMA_IsActiveFlag_TE5(DMA2)) 
        {
            LL_DMA_ClearFlag_TE5(DMA2);
            uart1_errorCallback();
        }
    }
    
    void DMA2_Stream7_IRQHandler(void) 
    {
        // TX DMA Interrupt
        if (LL_DMA_IsActiveFlag_TC7(DMA2)) 
        {
            LL_DMA_ClearFlag_TC7(DMA2);
            uart1_dma_transmit_complete_callback();
        }
        if (LL_DMA_IsActiveFlag_TE7(DMA2)) 
        {
            LL_DMA_ClearFlag_TE7(DMA2);
            uart1_errorCallback();
        }
    }
    
    void USART1_IRQHandler(void)
    {
        // UART Interrupt - IDLE line detection
        if (LL_USART_IsActiveFlag_IDLE(USART1)) 
        {
            LL_USART_ClearFlag_IDLE(USART1);
            uart1_dma_receive_handle();
            #if UART_I1_DMA_RECEIVE_COMPLETE_CALLBACK_ENABLED
            uart1_dma_receive_complete_callback();
            #endif
        }
        
        // Handle other UART errors
        if (LL_USART_IsActiveFlag_ORE(USART1) || 
            LL_USART_IsActiveFlag_FE(USART1) || 
            LL_USART_IsActiveFlag_NE(USART1)) {
            uart1_errorCallback();
        }
    }
}


#endif