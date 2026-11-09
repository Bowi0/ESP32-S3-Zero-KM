#ifndef UART_UPDATE_PROTOCOL_H
#define UART_UPDATE_PROTOCOL_H

#include "uart_switch_protocol.h"
#include <stddef.h>
#include <string.h>

#define UPDATE_MAX_PAYLOAD 256
#define UPDATE_HEADER_SIZE 9
#define UPDATE_FRAME_MAX (UPDATE_HEADER_SIZE + UPDATE_MAX_PAYLOAD + 2)
#define UPDATE_BEGIN 1
#define UPDATE_DATA 2
#define UPDATE_FINISH 3
#define UPDATE_ABORT 4
#define UPDATE_REPLY 5

typedef struct {
    uint8_t bytes[UPDATE_FRAME_MAX];
    uint16_t used;
    uint16_t expected;
} update_parser_t;

static inline uint32_t update_get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static inline void update_put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

static inline size_t update_encode(uint8_t *frame, uint8_t type, uint32_t offset,
                                   const uint8_t *payload, uint16_t length)
{
    if(length > UPDATE_MAX_PAYLOAD || (length && !payload)) return 0;
    frame[0] = 0xC3;
    frame[1] = 0x3C;
    frame[2] = type;
    update_put_u32(frame + 3, offset);
    frame[7] = (uint8_t)(length >> 8);
    frame[8] = (uint8_t)length;
    if(length) memcpy(frame + UPDATE_HEADER_SIZE, payload, length);
    const uint16_t crc = switch_crc16(frame, UPDATE_HEADER_SIZE + length);
    frame[UPDATE_HEADER_SIZE + length] = (uint8_t)(crc >> 8);
    frame[UPDATE_HEADER_SIZE + length + 1] = (uint8_t)crc;
    return UPDATE_HEADER_SIZE + length + 2;
}

static inline bool update_parse(update_parser_t *parser, uint8_t value)
{
    if(parser->used == 0 && value != 0xC3) return false;
    if(parser->used == 1 && value != 0x3C) {
        parser->used = value == 0xC3 ? 1 : 0;
        return false;
    }
    parser->bytes[parser->used++] = value;
    if(parser->used == UPDATE_HEADER_SIZE) {
        uint16_t length = ((uint16_t)parser->bytes[7] << 8) | parser->bytes[8];
        if(length > UPDATE_MAX_PAYLOAD) {
            parser->used = 0;
            return false;
        }
        parser->expected = UPDATE_HEADER_SIZE + length + 2;
    }
    if(parser->used < UPDATE_HEADER_SIZE || parser->used < parser->expected) return false;
    const uint16_t crc = switch_crc16(parser->bytes, parser->expected - 2);
    const bool valid = parser->bytes[parser->expected - 2] == (uint8_t)(crc >> 8) &&
                       parser->bytes[parser->expected - 1] == (uint8_t)crc;
    parser->used = 0;
    return valid;
}
#endif
