#include <assert.h>
#include <stdio.h>
#include "module_uart_mux.h"
#include "module_big_yaw_full.h"

static UartMux_State mux;
static BigYaw_FullStage stage;
static unsigned pc_count, vision_count, accepted;
static uint32_t now;
static uint8_t safe = 1;

static uint8_t crc(const uint8_t *p, unsigned n)
{
    uint8_t c = 0;
    for (unsigned i = 0; i < n; ++i) {
        c ^= p[i];
        for (unsigned j = 0; j < 8; ++j) c = (uint8_t)((c << 1) ^ ((c & 128) ? 0x31 : 0));
    }
    return c;
}

static uint8_t valid(const uint8_t *p, uint16_t n)
{
    return (uint8_t)(p[n-1] == (n == 16 ? 13 : 0x45) && p[n-2] == crc(p,n-2));
}

static void dispatch(const uint8_t *p, uint16_t n)
{
    if (n == 15) { ++vision_count; return; }
    ++pc_count;
    if (p[1] == 0x30 && BigYaw_FullReceive(&stage, p+2, now, safe) == 1) ++accepted;
}

static void consume(uint8_t *p, uint16_t n)
{
    UartMux_Push(&mux, p, n, now, valid, dispatch);
}

static void reset(void)
{
    memset(&mux, 0, sizeof(mux)); memset(&stage, 0, sizeof(stage));
    pc_count = vision_count = accepted = 0; now = 1; safe = 1;
}

static void packet(uint8_t *p, unsigned id, unsigned part)
{
    const float values[8] = {3, .6f, 4, .03f, 0, 0, 0, 0};
    memset(p, 0, 16);
    p[0] = 255; p[1] = 0x30; p[2] = (uint8_t)id; p[3] = (uint8_t)(id >> 8);
    p[4] = (uint8_t)part; p[5] = 0xC3;
    memcpy(p+6, values+part*2, 8);
    p[14] = crc(p,14); p[15] = 13;
}

static void ring_stream(uint8_t *data, unsigned len, unsigned idle_every)
{
    uint8_t ring[64] = {0};
    uint16_t cursor = 0;
    for (unsigned i = 0; i < len; ++i) {
        unsigned position = (i+1) % 64;
        ring[i % 64] = data[i];
        if (position == 32 || position == 0 || (idle_every && (i+1)%idle_every == 0)) {
            UartMux_Drain(&cursor, ring, 64, (uint16_t)position, consume);
            /* IDLE after HT/TC must not duplicate a frame or transaction. */
            UartMux_Drain(&cursor, ring, 64, (uint16_t)position, consume);
        }
    }
    UartMux_Drain(&cursor, ring, 64, (uint16_t)(len % 64), consume);
}

int main(void)
{
    uint8_t data[16*4*10], vision[15] = {0x53};
    for (unsigned i = 0; i < 40; ++i) packet(data+i*16, i/4+1, i%4);
    for (unsigned stride = 1; stride <= 65; ++stride) {
        reset();
        for (unsigned i = 0; i < sizeof(data); i += stride) {
            unsigned n = sizeof(data)-i < stride ? sizeof(data)-i : stride;
            consume(data+i, (uint16_t)n);
        }
        assert(pc_count == 40 && accepted == 10);
    }
    /* Reproduce the old IDLE-only loss at exactly 64 bytes (NDTR reloaded). */
    reset();
    { uint16_t cursor = 0; UartMux_Drain(&cursor, data, 64, 0, consume); }
    assert(pc_count == 0);
    for (unsigned idle = 0; idle <= 65; ++idle) {
        reset(); ring_stream(data, 64, idle); assert(pc_count == 4 && accepted == 1);
        reset(); ring_stream(data, sizeof(data), idle); assert(pc_count == 40 && accepted == 10);
    }
    reset();
    vision[13] = crc(vision,13); vision[14] = 0x45;
    consume(vision,7); consume(vision+7,8);
    consume(data,31); consume(vision,15); consume(data+31,33);
    assert(vision_count == 2 && accepted == 0); /* Corrupted PC frame cannot apply. */
    consume(data+64,64); assert(accepted == 1);
    reset(); consume(data,8); now += 101; consume(data+8,8);
    assert(pc_count == 0); consume(data+64,64); assert(accepted == 1);
    reset(); safe = 0; ring_stream(data,64,0); assert(pc_count == 4 && accepted == 0);
    reset();
    { uint8_t corrupt[16]; memcpy(corrupt,data,16); corrupt[7] ^= 1;
      consume(corrupt,16); assert(pc_count == 0); }
    consume(data+64,64); assert(accepted == 1);
    puts("PASS: UART fragmentation, CRC, mux, exact-64 DMA wrap, HT/TC/IDLE and full PID staging");
    return 0;
}
