#pragma once

template<uint8_t instance>
class Eth{
    public:
    static void init();
    static void send(char c);
    static void send(uint8_t len, char* c);
    static uint16_t recieve(uint8_t len, char* c, char term = '\n');
    static bool is_connected();

    // Optional: Buffer status (useful for flow control)
    static uint16_t tx_space_available();
    static uint16_t rx_bytes_available();
    
    // Optional: Async status
    static bool is_tx_complete();

    // Optional: Error handling
    static bool has_error();
    static void clear_errors();
};