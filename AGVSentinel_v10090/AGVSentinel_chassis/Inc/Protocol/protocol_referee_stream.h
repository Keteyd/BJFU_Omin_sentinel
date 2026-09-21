#ifndef PROTOCOL_REFEREE_STREAM_H
#define PROTOCOL_REFEREE_STREAM_H

#include "alg_crc.h"
#include <string.h>

typedef struct {
    uint8_t data[300];
    uint16_t used;
} Referee_Stream;

/* Retain partial frames across idle events; resync bad CRC byte by byte. */
static inline void Referee_StreamFeed(Referee_Stream *stream,
                                      const uint8_t *bytes, uint16_t count,
                                      void (*consume)(uint8_t *, uint16_t))
{
    uint16_t i;
    for (i = 0U; i < count; ++i) {
        if (stream->used == sizeof(stream->data)) {
            --stream->used;
            memmove(stream->data, stream->data + 1, stream->used);
        }
        stream->data[stream->used++] = bytes[i];
        while (stream->used != 0U) {
            uint16_t payload;
            uint16_t length;
            uint16_t remove = 1U;
            if (stream->data[0] == 0xA5U) {
                if (stream->used < 5U) break;
                if (CRC_VerifyCRC8CheckSum(stream->data, 5U)) {
                    payload = (uint16_t)stream->data[1] |
                              ((uint16_t)stream->data[2] << 8);
                    if (payload <= sizeof(stream->data) - 9U) {
                        length = payload + 9U;
                        if (stream->used < length) break;
                        if (CRC_VerifyCRC16CheckSum(stream->data, length)) {
                            consume(stream->data, length);
                            remove = length;
                        }
                    }
                }
            }
            stream->used -= remove;
            memmove(stream->data, stream->data + remove, stream->used);
        }
    }
}

#endif
