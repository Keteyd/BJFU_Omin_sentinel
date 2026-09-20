#include <assert.h>
#include <stdio.h>
#include "module_pitch_limits.h"
#include "protocol_referee_stream.h"

static unsigned received;
static void consume(uint8_t *frame, uint16_t length)
{
    assert(length >= 9 && length <= 300);
    assert(CRC_VerifyCRC16CheckSum(frame, length));
    ++received;
}

static void make_frame(uint8_t *frame, uint16_t length)
{
    memset(frame, 0x37, length);
    frame[0] = 0xA5;
    frame[1] = (uint8_t)(length - 9);
    frame[2] = (uint8_t)((length - 9) >> 8);
    CRC_AppendCRC8CheckSum(frame, 5);
    CRC_AppendCRC16CheckSum(frame, length);
}

int main(void)
{
    float lo, hi;
    const float rev = 6.283185307f;
    uint8_t frame[300], corrupt[300], garbage[400];
    Referee_Stream stream = {{0}, 0};
    unsigned i, split;
    int k;
    for (k = -1; k <= 1; ++k) {
        const float poses[] = {-0.455f, -0.1f, 0.259f};
        for (i = 0; i < 3; ++i) {
            assert(PitchLimits_Align(poses[i] + k * rev, -0.455f, 0.259f,
                                     0.005f, 12.5f, &lo, &hi));
            assert(fabsf(lo - (-0.450f + k * rev)) < 0.00001f);
            assert(fabsf(hi - (0.254f + k * rev)) < 0.00001f);
        }
    }
    assert(!PitchLimits_Align(2.0f, -0.455f, 0.259f, 0.005f, 12.5f, &lo, &hi));
    assert(!PitchLimits_Align(NAN, -0.455f, 0.259f, 0.005f, 12.5f, &lo, &hi));
    assert(!PitchLimits_Align(INFINITY, -0.455f, 0.259f, 0.005f, 12.5f, &lo, &hi));
    assert(!PitchLimits_Align(2 * rev - 0.2f, -0.455f, 0.259f, 0.005f, 12.5f, &lo, &hi));
    assert(!PitchLimits_Align(0, 0.259f, -0.455f, 0.005f, 12.5f, &lo, &hi));
    assert(!PitchLimits_Align(0, -4, 4, 0.005f, 12.5f, &lo, &hi));

    make_frame(frame, 30);
    for (split = 1; split < 30; ++split) {
        stream.used = 0;
        received = 0;
        Referee_StreamFeed(&stream, frame, split, consume);
        assert(received == 0);
        Referee_StreamFeed(&stream, frame + split, 30 - split, consume);
        assert(received == 1 && stream.used == 0);
    }
    memcpy(corrupt, frame, 30);
    corrupt[20] ^= 1;
    memset(garbage, 0xFF, sizeof(garbage));
    received = 0;
    Referee_StreamFeed(&stream, garbage, sizeof(garbage), consume);
    Referee_StreamFeed(&stream, corrupt, 30, consume);
    Referee_StreamFeed(&stream, frame, 30, consume);
    Referee_StreamFeed(&stream, frame, 30, consume);
    assert(received == 2 && stream.used == 0);
    make_frame(frame, 300);
    for (i = 0; i < 300; ++i) Referee_StreamFeed(&stream, frame + i, 1, consume);
    assert(received == 3 && stream.used == 0);
    frame[1] = 0xFF;
    frame[2] = 0xFF;
    CRC_AppendCRC8CheckSum(frame, 5);
    Referee_StreamFeed(&stream, frame, 300, consume);
    assert(received == 3);
    puts("PASS: pitch branch/limits and referee CRC/fragmentation regressions");
    return 0;
}
