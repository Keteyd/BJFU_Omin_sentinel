#include <assert.h>
#include <stdio.h>
#include "module_yaw_identification.h"

static YawIdent_Observation observation(void)
{
    YawIdent_Observation o = {0};
    o.now_ms = 100U;
    o.big_deg = 359.0f;
    o.small_joint_deg = 0.0f;
    o.imu_yaw_deg = 720.0f;
    o.capture_capacity = YAW_IDENT_SAMPLES;
    o.feedback_valid = o.remote_up = o.operator_neutral = 1U;
    o.yaw_outputs_off = o.capture_ready = 1U;
    return o;
}

static YawIdent_Request request(uint8_t axis)
{
    YawIdent_Request r = {1U, 3.0f, axis, 0U, 0U};
    return r;
}

static void run_axis(uint8_t axis, uint32_t start)
{
    YawIdent_State s;
    YawIdent_Observation o = observation();
    YawIdent_Request r = request(axis);
    YawIdent_Result out;
    unsigned i;
    float peak = 0.0f, sum = 0.0f;
    YawIdent_Init(&s);
    o.now_ms = start;
    assert(YawIdent_Arm(&s, &r, &o));
    assert(!YawIdent_Step(&s, &o).reference_valid);
    o.remote_up = 0U; o.remote_down = 1U;
    o.now_ms += 4U;
    out = YawIdent_Step(&s, &o);
    assert(out.phase == YAW_IDENT_BASELINE && out.reference_valid);
    for (i = 0; i <= YAW_IDENT_DURATION_MS; i += 4U) {
        float value;
        o.now_ms = start + 4U + i;
        if (i % 100U == 0U) assert(YawIdent_Heartbeat(&s, 1U, o.now_ms));
        out = YawIdent_Step(&s, &o);
        assert(out.phase != YAW_IDENT_ABORTED);
        assert(out.big_anchor_deg == 359.0f && out.small_heading_anchor_deg == 720.0f);
        assert(axis == 1U ? out.small_heading_offset_deg == 0.0f : out.big_offset_deg == 0.0f);
        value = out.big_offset_deg + out.small_heading_offset_deg;
        assert(fabsf(value) <= r.amplitude_deg + 1e-5f);
        peak = fmaxf(peak, fabsf(value)); sum += value;
    }
    assert(peak > 1.0f && fabsf(sum) < 0.01f);
    assert(out.phase == YAW_IDENT_DONE && !out.reference_valid);
    assert(!YawIdent_Heartbeat(&s, 1U, o.now_ms));
    assert(!YawIdent_Step(&s, &o).reference_valid);
    o.remote_up = 1U; o.remote_down = 0U;
    assert(!YawIdent_Arm(&s, &r, &o));
    r.trial_id = 2U;
    assert(YawIdent_Arm(&s, &r, &o));
}

static void rejects(void)
{
    unsigned failure;
    for (failure = 0; failure < 12U; ++failure) {
        YawIdent_State s;
        YawIdent_Observation o = observation();
        YawIdent_Request r = request(1U);
        YawIdent_Init(&s);
        switch (failure) {
        case 0: o.imu_age_ms = 11U; break;
        case 1: o.remote_down = 1U; break;
        case 2: o.yaw_outputs_off = 0U; break;
        case 3: o.small_joint_deg = 11.0f; break;
        case 4: o.big_rate_dps = 7.0f; break;
        case 5: o.capture_capacity = YAW_IDENT_SAMPLES - 1U; break;
        case 6: r.amplitude_deg = NAN; break;
        case 7: r.amplitude_deg = 6.0f; break;
        case 8: r.axis = 3U; break;
        case 9: r.trial_id = 0U; break;
        case 10: o.big_deg = NAN; break;
        default: o.remote_age_ms = 51U; break;
        }
        {
            uint8_t rejected = (uint8_t)(failure == 2U || failure == 5U ||
                failure == 6U || failure == 7U || failure == 8U || failure == 9U);
            assert(YawIdent_Arm(&s, &r, &o) == !rejected);
            if (rejected) assert(s.phase == YAW_IDENT_IDLE && s.last_id == 0U);
        }
    }
}

static void aborts(void)
{
    unsigned failure;
    for (failure = 0; failure < 9U; ++failure) {
        YawIdent_State s;
        YawIdent_Observation o = observation();
        YawIdent_Request r = request(2U);
        YawIdent_Result out;
        YawIdent_Init(&s);
        assert(YawIdent_Arm(&s, &r, &o));
        o.now_ms += 4U; o.remote_up = 0U; o.remote_down = 1U;
        assert(YawIdent_Step(&s, &o).reference_valid);
        o.now_ms += 4U;
        switch (failure) {
        case 0: o.remote_up = 1U; o.remote_down = 0U; break;
        case 1: o.remote_down = 0U; break;
        case 2: o.operator_neutral = 0U; break;
        case 3: o.small_age_ms = 21U; break;
        case 4: o.small_joint_deg = 13.0f; break;
        case 5: o.big_rate_dps = 361.0f; break;
        case 6: o.capture_ready = 0U; break;
        case 7: o.now_ms += 11U; break;
        default: o.imu_yaw_deg = INFINITY; break;
        }
        out = YawIdent_Step(&s, &o);
        assert(out.phase == YAW_IDENT_ABORTED && !out.reference_valid);
        o = observation();
        assert(!YawIdent_Step(&s, &o).reference_valid);
        assert(!YawIdent_Arm(&s, &r, &o));
    }
}

static void expiry(void)
{
    YawIdent_State s;
    YawIdent_Observation o = observation();
    YawIdent_Request r = request(1U);
    unsigned i;
    YawIdent_Init(&s);
    assert(YawIdent_Arm(&s, &r, &o));
    assert(!YawIdent_Heartbeat(&s, 2U, o.now_ms));
    for (i = 0; i < 304U; i += 4U) { o.now_ms += 4U; (void)YawIdent_Step(&s, &o); }
    assert(s.phase == YAW_IDENT_ABORTED && s.reason == YAW_IDENT_LINK_LOST);
    assert(!YawIdent_Heartbeat(&s, 1U, o.now_ms));
    r.trial_id = 2U;
    assert(YawIdent_Arm(&s, &r, &o));
    for (i = 0; i <= YAW_IDENT_ARM_TIMEOUT_MS; i += 4U) {
        o.now_ms += 4U;
        if (i % 100U == 0U) assert(YawIdent_Heartbeat(&s, 2U, o.now_ms));
        (void)YawIdent_Step(&s, &o);
    }
    assert(s.phase == YAW_IDENT_ABORTED && s.reason == YAW_IDENT_ARM_EXPIRED);
}

static void rate_boundaries(void)
{
    int sign;
    for (sign = -1; sign <= 1; sign += 2) {
        YawIdent_State s;
        YawIdent_Observation o = observation();
        YawIdent_Request r = request(1U);
        YawIdent_Init(&s);
        assert(YawIdent_Arm(&s, &r, &o));
        o.remote_up = 0U; o.remote_down = 1U; o.now_ms += 4U;
        assert(YawIdent_Step(&s, &o).reference_valid);
        o.big_rate_dps = (float)sign * 360.0f; o.now_ms += 4U;
        assert(YawIdent_Step(&s, &o).reference_valid);
        o.big_rate_dps = (float)sign * 361.0f; o.now_ms += 4U;
        assert(!YawIdent_Step(&s, &o).reference_valid);
        assert(s.phase == YAW_IDENT_ABORTED && s.reason == YAW_IDENT_TRAVEL);
        o = observation();
        o.small_rate_dps = (float)sign * 180.0f;
        assert(YawIdent_InsideTravel(&o));
        o.small_rate_dps = (float)sign * 181.0f;
        assert(!YawIdent_InsideTravel(&o));
    }
}

int main(void)
{
    assert(YawIdent_Wave(0U, 3.0f) == 0.0f);
    assert(YawIdent_Wave(YAW_IDENT_DRIVE_MS, 3.0f) == 0.0f);
    run_axis(1U, 100U);
    run_axis(2U, 0xFFFFFF00U);
    rejects(); aborts(); expiry(); rate_boundaries();
    puts("yaw identification core tests passed (no hardware integration)");
    return 0;
}
