#ifndef UART_SWITCH_PROTOCOL_H
#define UART_SWITCH_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define SWITCH_FRAME_SIZE 7
#define SWITCH_SET 1
#define SWITCH_ACK 2
#define SWITCH_VERSION_REQUEST 3
#define SWITCH_VERSION_MAJOR 4
#define SWITCH_VERSION_MINOR 5
#define SWITCH_VERSION_PATCH 6
#define SWITCH_PRESSURE 7
#define SWITCH_PRESSURE_STREAM 8
#define SWITCH_PRESSURE_MAX_CENTIBAR 1000

static inline uint16_t switch_crc16(const uint8_t *data, unsigned length)
{
    uint16_t crc = 0xFFFF;
    for(unsigned i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for(unsigned bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static inline void switch_encode(uint8_t *frame, uint8_t type, uint8_t seq, uint8_t mask)
{
    frame[0] = 0xA5; frame[1] = 0x5A; frame[2] = type; frame[3] = seq; frame[4] = mask;
    const uint16_t crc = switch_crc16(frame, 5);
    frame[5] = (uint8_t)(crc >> 8); frame[6] = (uint8_t)crc;
}

typedef struct { uint8_t data[SWITCH_FRAME_SIZE]; uint8_t used; } switch_parser_t;

static inline bool switch_parse(switch_parser_t *parser, uint8_t value)
{
    if(parser->used == 0 && value != 0xA5) return false;
    if(parser->used == 1 && value != 0x5A) {
        parser->used = value == 0xA5 ? 1 : 0;
        return false;
    }
    parser->data[parser->used++] = value;
    if(parser->used < SWITCH_FRAME_SIZE) return false;
    parser->used = 0;
    const uint16_t crc = switch_crc16(parser->data, 5);
    if(parser->data[5] == (uint8_t)(crc >> 8) &&
       parser->data[6] == (uint8_t)crc) return true;

    for(unsigned i = 1; i < SWITCH_FRAME_SIZE; i++) {
        if(parser->data[i] == 0xA5 &&
           (i == SWITCH_FRAME_SIZE - 1 || parser->data[i + 1] == 0x5A)) {
            parser->used = (uint8_t)(SWITCH_FRAME_SIZE - i);
            for(unsigned j = 0; j < parser->used; j++)
                parser->data[j] = parser->data[i + j];
            break;
        }
    }
    return false;
}
#endif
