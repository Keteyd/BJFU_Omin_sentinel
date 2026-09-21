#ifndef MODULE_UART_MUX_H
#define MODULE_UART_MUX_H

#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t bytes[16];
    uint8_t used;
    uint32_t last_ms;
} UartMux_State;

typedef uint8_t (*UartMux_Validate)(const uint8_t *, uint16_t);
typedef void (*UartMux_Dispatch)(const uint8_t *, uint16_t);

/* Invalid frame candidates resync one byte at a time, retaining partial frames. */
static inline void UartMux_Push(UartMux_State *s, const uint8_t *data,
    uint16_t len, uint32_t now, UartMux_Validate valid, UartMux_Dispatch dispatch)
{
    uint16_t i;
    if (!data) return;
    if (now - s->last_ms > 100U) s->used = 0U;
    s->last_ms = now;
    for (i = 0; i < len; ++i) {
        s->bytes[s->used++] = data[i];
        while (s->used) {
            uint16_t needed = s->bytes[0] == 0xFFU ? 16U :
                (s->bytes[0] == 0x53U ? 15U : 0U);
            if (needed && s->used < needed) break;
            if (needed && valid(s->bytes, needed)) {
                dispatch(s->bytes, needed);
                s->used = (uint8_t)(s->used - needed);
                memmove(s->bytes, s->bytes + needed, s->used);
            } else {
                --s->used;
                memmove(s->bytes, s->bytes + 1, s->used);
            }
        }
    }
}

/* HT, TC and IDLE share one cursor. Service HT/TC before DMA laps the reader. */
typedef void (*UartMux_Consume)(uint8_t *, uint16_t);
static inline void UartMux_Drain(uint16_t *cursor, uint8_t *buffer,
    uint16_t size, uint16_t position, UartMux_Consume consume)
{
    if (!size || position > size || *cursor >= size) return;
    if (position < *cursor) {
        consume(buffer + *cursor, (uint16_t)(size - *cursor));
        *cursor = 0U;
    }
    if (position > *cursor) consume(buffer + *cursor, (uint16_t)(position - *cursor));
    *cursor = position == size ? 0U : position;
}

#endif
