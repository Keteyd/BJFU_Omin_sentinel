#include <assert.h>
#include <stdio.h>
#include <float.h>
#include "module_big_yaw_full.h"

static void packet(uint8_t p[12], uint16_t id, uint8_t part, const float v[8])
{
    p[0] = (uint8_t)id; p[1] = (uint8_t)(id >> 8);
    p[2] = part; p[3] = 0xC3;
    memcpy(p + 4, &v[part * 2], 8);
}

int main(void)
{
    BigYaw_FullStage s = {0};
    float v[8] = {100000, 20000, 30, 1000, .01f, .3f, .02f, .4f};
    uint8_t p[12];
    unsigned i;
    assert(BigYaw_FullValid(v));
    for (i = 0; i < 4; ++i) {
        packet(p, 123, (uint8_t)i, v);
        assert(BigYaw_FullReceive(&s, p, i, 1) == (i == 3 ? 1 : 0));
    }
    assert(memcmp(s.values, v, sizeof(v)) == 0);
    assert(BigYaw_FullReceive(&s, p, 5, 1) == 0);
    packet(p, 123, 0, v);
    assert(BigYaw_FullReceive(&s, p, 6, 1) == 0);
    packet(p, 124, 0, v); assert(BigYaw_FullReceive(&s, p, 10, 1) == 0);
    packet(p, 124, 2, v); assert(BigYaw_FullReceive(&s, p, 11, 1) == 3);
    packet(p, 124, 3, v); assert(BigYaw_FullReceive(&s, p, 12, 1) == 0);
    packet(p, 125, 0, v); assert(BigYaw_FullReceive(&s, p, 20, 1) == 0);
    packet(p, 125, 1, v); assert(BigYaw_FullReceive(&s, p, 21, 0) == 2);
    packet(p, 125, 2, v); assert(BigYaw_FullReceive(&s, p, 22, 1) == 0);
    packet(p, 126, 0, v); assert(BigYaw_FullReceive(&s, p, 30, 1) == 0);
    packet(p, 126, 1, v); assert(BigYaw_FullReceive(&s, p, 331, 1) == 4);
    packet(p, 127, 0, v); assert(BigYaw_FullReceive(&s, p, UINT32_MAX, 1) == 0);
    packet(p, 127, 1, v); assert(BigYaw_FullReceive(&s, p, 0, 1) == 0);
    packet(p, 128, 2, v); assert(BigYaw_FullReceive(&s, p, 1, 1) == 3);
    packet(p, 129, 0, v); p[3] = 0; assert(BigYaw_FullReceive(&s, p, 2, 1) == 3);
    packet(p, 0, 0, v); assert(BigYaw_FullReceive(&s, p, 3, 1) == 3);
    for (i = 0; i < 8; ++i) {
        float previous = v[i];
        v[i] = NAN; assert(!BigYaw_FullValid(v));
        v[i] = INFINITY; assert(!BigYaw_FullValid(v));
        v[i] = -1; assert(!BigYaw_FullValid(v));
        v[i] = previous;
    }
    v[0] = v[1] = v[3] = v[4] = v[5] = v[6] = v[7] = FLT_MAX;
    assert(BigYaw_FullValid(v));
    v[2] = 30.001f; assert(!BigYaw_FullValid(v));
    v[2] = 0; assert(BigYaw_FullValid(v));
    v[7] = NAN;
    for (i = 0; i < 4; ++i) {
        packet(p, 200, (uint8_t)i, v);
        assert(BigYaw_FullReceive(&s, p, 400 + i, 1) == (i == 3 ? 3 : 0));
    }
    puts("PASS: full PID float validation, ordered atomic staging, expiry, SAFE and replay rejection");
    return 0;
}
