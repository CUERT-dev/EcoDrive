#pragma once

//==================DESCRIPTION============================
/*
    
    AEBFV0 (Asynchronous Expandable Binary frame version 0)

    Frame format:
    [0x00 SYNC_BYTE][1Byte Payload Size][1Byte MCOBS_CODE][MCOBS_ENCODED([6Bit DeviceID][10Bit ServiceID][Payload][CRC16])]
    
    CRC if for entire frame including all bytes except the SYNC_BYTE
    total Frame encoding overhead
    1(0x00) + 1(Payload Size) + 2(DeviceID+ServiceID) + 1(MCOBS firstCodeByte) + 2(CRC16) = 7
*/
#include <stdint.h>
#include <stdbool.h>

// ================== FUNCTION PROTOTYPES ==================
#define AEBP_FRAMING_BYTESNUM 7
#define AEBP_FRAMED_SIZE(s)(s+AEBP_FRAMING_BYTESNUM)
// ================== CONSTANTS ==================
#define AEBP_SYNC_BYTE          0x00
#define AEBP_MAX_PAYLOAD_SIZE   245
#define AEBP_MAX_DEVICES        63      // 6 bits = 63
#define AEBP_MAX_SERVICES       1023    // 10 bits = 1023


#define AEBP_FRAME_OK 0
#define AEBP_FRAME_INCOMPLETE 1
#define AEBP_FRAME_CORRUPTED 2

#ifdef __cplusplus
extern "C" {
#endif

bool aebp_is_frame_cplt(uint8_t* frame_buffer, uint16_t frame_len);

uint8_t aebp_encode_frame(uint8_t* output_buffer,
                            uint8_t device_id, uint16_t service_id,
                            const uint8_t* payload, uint8_t payload_len);

uint8_t aebp_decode_frame(uint8_t* frame_buffer, uint16_t frame_len,
                            uint8_t *device_id, uint16_t *service_id,
                            uint8_t* payload, uint8_t *payload_len);


#ifdef __cplusplus
}
#endif