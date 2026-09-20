#include <assert.h>
#include <stdio.h>
#include "module_yaw_burst.h"

static YawBurst_Buffer b;
int main(void)
{
    uint8_t payload[12], previous[12];
    YawBurst_Reset(&b);
    assert(!YawBurst_Payload(&b, payload));
    assert(YawBurst_Due(&b, UINT32_MAX));
    assert(!YawBurst_Due(&b, 0));
    assert(YawBurst_Due(&b, 1));
    assert(YawBurst_Due(&b, 7) && b.skipped == 2U);
    YawBurst_Reset(&b);
    for (unsigned i = 0; i < YAW_BURST_COUNT; ++i) {
        assert(YawBurst_Due(&b, i * 2U));
        b.records[i].observation.tick_ms = i * 2U;
        b.records[i].observation.imu_sequence = i + 1U;
        b.records[i].observation.flags = 207U;
        b.records[i].observation.imu_dt_s = .001f;
        b.records[i].observation.big_encoder_deg = 350.0f;
        b.records[i].observation.small_encoder_deg = 300.0f;
        b.records[i].control_dt_s = .002f;
        b.records[i].gimbal_dt_s = .001f;
        ++b.count;
        if (i + 1U < YAW_BURST_COUNT) assert(!YawBurst_Payload(&b, payload));
    }
    assert(!YawBurst_Due(&b, 100000U));
    while (YawBurst_Payload(&b, payload)) {
        memcpy(previous, payload, sizeof(payload));
        assert(YawBurst_Payload(&b, payload));
        assert(memcmp(previous, payload, sizeof(payload)) == 0);
        /* Hex payload fixture for Python's independent decoder. */
        for (unsigned i = 0; i < sizeof(payload); ++i) printf("%02x", payload[i]);
        putchar('\n');
        YawBurst_Accept(&b);
    }
    assert(b.sent == YAW_BURST_COUNT && b.part == 0U);
    YawBurst_Accept(&b);
    assert(b.sent == YAW_BURST_COUNT);
    YawBurst_Reset(&b);
    assert(b.count == 0U && b.sent == 0U && !b.started);
    return 0;
}
