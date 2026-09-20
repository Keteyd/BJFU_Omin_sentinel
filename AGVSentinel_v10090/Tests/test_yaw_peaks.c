#include <assert.h>
#include <stdio.h>
#include "module_yaw_peaks.h"

static void update(YawPeaks_State *s, uint32_t t, uint16_t raw)
{
    YawPeaks_Update(s, t, 1, 1, raw, -2.5f, -3.0f, -4.0f, 0);
}

int main(void)
{
    YawPeaks_State s;
    uint16_t packet[6];
    unsigned i;
    assert(sizeof(packet) == 12);
    YawPeaks_Reset(&s);
    update(&s, 0, 8191);
    update(&s, 1, 0);
    update(&s, 2, 100);
    update(&s, 3, 8191);
    YawPeaks_Pack(&s, 4, packet);
    assert(packet[0] == 4 && packet[1] == 101);
    assert(packet[2] == 250 && packet[3] == 300 && packet[4] == 4000);
    assert(packet[5] == 4);
    /* Packing without successful TX must not clear the peaks. */
    YawPeaks_Pack(&s, 5, packet);
    assert(packet[1] == 101);
    YawPeaks_NextWindow(&s, 5);
    update(&s, 6, 0);
    assert(s.span == 1 && s.samples == 1);
    YawPeaks_Update(&s, 7, 0, 1, 2000, 0, 0, 0, 0);
    update(&s, 8, 3000);
    assert(s.span == 1 && s.samples == 2);
    YawPeaks_Update(&s, 9, 1, 1, 3001, 0, 0, 0, 1);
    YawPeaks_Pack(&s, 10, packet);
    assert(packet[0] == (0x4000U | 3U));
    YawPeaks_Update(&s, 10, 1, 0, 3002, 0, 0, 0, 0);
    YawPeaks_Pack(&s, 10, packet);
    assert(packet[0] & 0x8000U);
    YawPeaks_Reset(&s);
    update(&s, UINT32_MAX, 100);
    update(&s, 0, 99);
    YawPeaks_Pack(&s, 1, packet);
    assert(packet[5] == 2 && !(packet[0] & 0x8000U));
    update(&s, 11, 98);
    assert(s.invalid);
    YawPeaks_Reset(&s);
    YawPeaks_Update(&s, 0, 1, 1, 1, NAN, 0, 0, 0);
    assert(s.invalid && s.samples == 0);
    YawPeaks_Reset(&s);
    YawPeaks_Update(&s, 0, 1, 1, 8192, 0, 0, 0, 0);
    assert(s.invalid && s.samples == 0);
    YawPeaks_Reset(&s);
    YawPeaks_Update(&s, 0, 1, 1, 1, 0, 700, 0, 0);
    assert(s.invalid && s.speed == 65535);
    YawPeaks_Reset(&s);
    for (i = 0; i < 20000; ++i) update(&s, i, 1);
    assert(s.samples == 0x3FFFU && s.invalid);
    YawPeaks_Reset(&s);
    for (i = 0; i < 100; ++i) update(&s, i, (uint16_t)((i * 3000) % 8192));
    assert(s.span == 65535 && s.invalid);
    YawPeaks_Pack(&s, 70000, packet);
    assert(packet[5] == 65535 && (packet[0] & 0x8000U));
    YawPeaks_Reset(&s);
    YawPeaks_Update(&s, 0, 0, 1, 100, 0, 0, 0, 0);
    YawPeaks_Pack(&s, 100, packet);
    assert(packet[0] == 0 && packet[5] == 100);
    puts("yaw peak diagnostics: all tests passed");
    return 0;
}
