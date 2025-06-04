#ifndef MTI_DECODE_H
#define MTI_DECODE_H

typedef enum
{
    OPEN_PORT_FAILURE = 100,
    OPEN_PORT_SUCCESS,
    PORT_SET_ATTR_FAILURE,
    PORT_SET_ATTR_SUCCESS,
    PORT_GET_ATTR_FAILURE,
    PORT_GET_ATTR_SUCCESS,
    DEVICE_FOUND_FAILURE,
    DEVICE_FOUND_SUCCESS,
} mtiDecode_enum;

typedef struct
{
    uint8_t preamble;     // Should be 0xFA
    uint8_t bid;          // Bus ID
    uint8_t mid;          // Message ID
    uint8_t len;          // Standard length or 0xFF for extended
    uint16_t ext_len;     // Optional: if len == 0xFF
    uint8_t ind_id;       // Optional: for indication messages
    std::vector<uint8_t> data;     // Payload
    uint8_t checksum;     // Checksum at end of message
} XbusMessage;

#endif // MTI_DECODE_H
