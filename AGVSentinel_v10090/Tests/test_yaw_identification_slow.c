#include <assert.h>
#include <stdio.h>
#include "module_yaw_identification.h"
#include "module_yaw_ident_wire.h"

static YawIdent_Observation observation(uint32_t tick)
{
    YawIdent_Observation o = {0};
    o.now_ms = tick; o.big_deg = 359.0f; o.imu_yaw_deg = 720.0f;
    o.capture_capacity = YAW_IDENT_SAMPLES;
    o.feedback_valid = o.remote_up = o.operator_neutral = o.yaw_outputs_off = o.capture_ready = 1U;
    return o;
}

static void full_profile(uint8_t axis, uint8_t reverse, uint8_t bench)
{
    YawIdent_State s = {0};
    YawIdent_Request r = {1U, axis == 1U ? 15.0f : 10.0f, axis, YAW_IDENT_PROFILE_SLOW, reverse};
    YawIdent_Observation o = observation(0xfffffff0U);
    uint32_t start, t;
    float peak = 0.0f;
    if (bench) { r.profile = YAW_IDENT_PROFILE_BENCH; r.axis = 0U; r.amplitude_deg = 0.0f; r.reverse = 0U; }
    assert(YawIdent_Arm(&s, &r, &o));
    if (!bench) { o.remote_up = 0U; o.remote_down = 1U; }
    start = o.now_ms;
    for (t = 0U; t <= 20000U; t += 4U) {
        YawIdent_Result result;
        o.now_ms = start + t;
        if (t % 100U == 0U) assert(YawIdent_Heartbeat(&s, 1U, o.now_ms));
        result = YawIdent_Step(&s, &o);
        assert(result.phase != YAW_IDENT_ABORTED);
        assert(result.reference_valid == (!bench && t < 20000U));
        if (t < 2000U) assert(result.phase == YAW_IDENT_BASELINE);
        else if (t < 16000U) assert(result.phase == YAW_IDENT_EXCITE);
        else if (t < 20000U) assert(result.phase == YAW_IDENT_SETTLE);
        else assert(result.phase == YAW_IDENT_DONE);
        if (axis == 1U) assert(result.small_heading_offset_deg == 0.0f);
        else assert(result.big_offset_deg == 0.0f);
        if (t == 5500U || t == 12500U) {
            float expected = r.amplitude_deg * (t == 5500U ? 1.0f : -1.0f) * (reverse ? -1.0f : 1.0f);
            assert(result.big_offset_deg + result.small_heading_offset_deg == expected);
        }
        peak = fmaxf(peak, fabsf(result.big_offset_deg + result.small_heading_offset_deg));
    }
    assert(peak == r.amplitude_deg);
}

static void bounds_and_bench(void)
{
    unsigned i;
    for (i = 0U; i < 9U; ++i) {
        YawIdent_State s = {0};
        YawIdent_Request r = {1U, 15.0f, 1U, YAW_IDENT_PROFILE_SLOW, 0U};
        YawIdent_Observation o = observation(100U);
        switch (i) {
        case 0: r.amplitude_deg = 15.01f; break;
        case 1: r.axis = 2U; r.amplitude_deg = 10.01f; break;
        case 2: o.small_joint_deg = 10.01f; break;
        case 3: o.small_joint_deg = -10.01f; break;
        case 4: r.profile = 3U; break;
        case 5: r.reverse = 2U; break;
        case 6: r.profile = YAW_IDENT_PROFILE_BENCH; break;
        case 7: r.amplitude_deg = NAN; break;
        default: r.profile = YAW_IDENT_PROFILE_SHORT; break;
        }
        assert(YawIdent_Arm(&s, &r, &o) == (i == 2U || i == 3U));
    }
    for (i = 0U; i < 4U; ++i) {
        YawIdent_State s = {0};
        YawIdent_Request r = {1U, 0.0f, 0U, YAW_IDENT_PROFILE_BENCH, 0U};
        YawIdent_Observation o = observation(100U);
        assert(YawIdent_Arm(&s, &r, &o));
        assert(!YawIdent_Step(&s, &o).reference_valid);
        assert(s.phase == YAW_IDENT_BASELINE);
        switch (i) {
        case 0: o.remote_up = 0U; o.remote_down = 1U; break;
        case 1: o.yaw_outputs_off = 0U; break;
        case 2: o.operator_neutral = 0U; break;
        default: o.feedback_valid = 0U; break;
        }
        o.now_ms += 4U;
        assert(!YawIdent_Step(&s, &o).reference_valid && s.phase == YAW_IDENT_ABORTED);
    }
    for (i = 0U; i < 3U; ++i) {
        YawIdent_State s = {0};
        YawIdent_Request r = {1U, 15.0f, 1U, YAW_IDENT_PROFILE_SLOW, 0U};
        YawIdent_Observation o = observation(100U);
        assert(YawIdent_Arm(&s, &r, &o));
        o.remote_up = 0U; o.remote_down = 1U;
        assert(YawIdent_Step(&s, &o).reference_valid);
        if (i == 0U) o.big_deg = 25.0f; /* +26 degrees across wrap. */
        if (i == 1U) o.small_joint_deg = -20.1f;
        if (i == 2U) o.imu_yaw_deg += 20.1f;
        o.now_ms += 4U;
        assert(!YawIdent_Step(&s, &o).reference_valid && s.reason == YAW_IDENT_TRAVEL);
    }
}

static void dual_profile(uint8_t phase_set)
{
    YawIdent_State s = {0};
    YawIdent_Request r = {1U, 0.0f, 3U, YAW_IDENT_PROFILE_DUAL, phase_set};
    YawIdent_Observation o = observation(100U);
    uint32_t start = o.now_ms, t;
    unsigned axis, positive[2] = {0U}, negative[2] = {0U};
    float peak[2] = {0.0f};
    assert(YawIdent_Arm(&s, &r, &o));
    o.remote_up = 0U; o.remote_down = 1U;
    for (t = 0U; t <= 20000U; t += 4U) {
        YawIdent_Result result;
        float value[2];
        o.now_ms = start + t;
        if (t % 100U == 0U) assert(YawIdent_Heartbeat(&s, 1U, o.now_ms));
        result = YawIdent_Step(&s, &o);
        assert(result.phase != YAW_IDENT_ABORTED);
        value[0] = result.big_offset_deg; value[1] = result.small_heading_offset_deg;
        if (t < 2000U || t >= 16000U) assert(value[0] == 0.0f && value[1] == 0.0f);
        for (axis = 0U; axis < 2U; ++axis) {
            peak[axis] = fmaxf(peak[axis], fabsf(value[axis]));
            if (value[axis] > 0.05f) ++positive[axis];
            if (value[axis] < -0.05f) ++negative[axis];
        }
    }
    assert(peak[0] <= YAW_IDENT_DUAL_BIG_PEAK_DEG && peak[0] > 2.0f);
    assert(peak[1] <= YAW_IDENT_DUAL_SMALL_PEAK_DEG && peak[1] > 1.0f);
    assert(positive[0] > 500U && negative[0] > 500U);
    assert(positive[1] > 500U && negative[1] > 500U);
}

static void cd_profile(uint8_t phase_set)
{
    YawIdent_State s = {0};
    YawIdent_Request r = {1U, 0.0f, 4U, YAW_IDENT_PROFILE_CD, phase_set};
    YawIdent_Observation o = observation(100U);
    uint32_t start = o.now_ms, t;
    unsigned axis, positive[2] = {0U}, negative[2] = {0U};
    const float scale[3] = {1.0f, 4.0f, 5.0f};
    float peak[2] = {0.0f}, previous[2] = {0.0f}, previous_rate[2] = {0.0f};
    float maximum_rate[2] = {0.0f}, maximum_acceleration[2] = {0.0f};
    assert(YawIdent_Arm(&s, &r, &o));
    assert(YawIdent_Duration(&r) == 36000U && YawIdent_Samples(&r) == 9001U);
    o.remote_up = 0U; o.remote_down = 1U;
    for (t = 0U; t <= YAW_IDENT_CD_DURATION_MS; t += 4U) {
        YawIdent_Result result;
        float value[2];
        o.now_ms = start + t;
        if (t % 100U == 0U) assert(YawIdent_Heartbeat(&s, 1U, o.now_ms));
        result = YawIdent_Step(&s, &o);
        assert(result.phase != YAW_IDENT_ABORTED);
        value[0] = result.big_offset_deg; value[1] = result.small_heading_offset_deg;
        if (t < 3000U || t >= 31000U) assert(value[0] == 0.0f && value[1] == 0.0f);
        if (t < 3000U) assert(result.phase == YAW_IDENT_BASELINE);
        else if (t < 31000U) assert(result.phase == YAW_IDENT_EXCITE);
        else if (t < 36000U) assert(result.phase == YAW_IDENT_SETTLE);
        else assert(result.phase == YAW_IDENT_DONE);
        for (axis = 0U; axis < 2U; ++axis) {
            float rate = t ? (value[axis] - previous[axis]) / 0.004f : 0.0f;
            float acceleration = t > 4U ? (rate - previous_rate[axis]) / 0.004f : 0.0f;
            peak[axis] = fmaxf(peak[axis], fabsf(value[axis]));
            maximum_rate[axis] = fmaxf(maximum_rate[axis], fabsf(rate));
            maximum_acceleration[axis] = fmaxf(maximum_acceleration[axis], fabsf(acceleration));
            if (value[axis] > 0.05f) ++positive[axis];
            if (value[axis] < -0.05f) ++negative[axis];
            previous[axis] = value[axis]; previous_rate[axis] = rate;
        }
    }
    assert(peak[0] <= 3.0f * scale[phase_set] && peak[0] > 2.7f * scale[phase_set]);
    assert(peak[1] <= 2.0f * scale[phase_set] && peak[1] > 1.6f * scale[phase_set]);
    assert(maximum_rate[0] < 12.0f * scale[phase_set]);
    assert(maximum_rate[1] < 12.0f * scale[phase_set]);
    assert(maximum_acceleration[0] < 200.0f * scale[phase_set]);
    assert(maximum_acceleration[1] < 200.0f * scale[phase_set]);
    assert(positive[0] > 1000U && negative[0] > 1000U);
    assert(positive[1] > 1000U && negative[1] > 1000U);
}

static void speed_profile(uint8_t phase_set)
{
    YawIdent_State s = {0};
    YawIdent_Request r = {1U, 0.0f, 5U, YAW_IDENT_PROFILE_SPEED, phase_set};
    YawIdent_Observation o = observation(100U);
    uint32_t start = o.now_ms, t;
    float peak[2] = {0.0f}, integral[2] = {0.0f};
    unsigned positive[2] = {0U}, negative[2] = {0U};
    assert(YawIdent_Arm(&s, &r, &o));
    assert(YawIdent_Duration(&r) == 36000U && YawIdent_Samples(&r) == 9001U);
    o.remote_up = 0U; o.remote_down = 1U;
    for (t = 0U; t <= YAW_IDENT_SPEED_DURATION_MS; t += 4U) {
        YawIdent_Result result;
        float value[2];
        unsigned axis;
        o.now_ms = start + t;
        if (t % 100U == 0U) assert(YawIdent_Heartbeat(&s, 1U, o.now_ms));
        result = YawIdent_Step(&s, &o);
        assert(result.phase != YAW_IDENT_ABORTED);
        value[0] = result.big_offset_deg; value[1] = result.small_heading_offset_deg;
        if (t < 3000U || t >= 31000U) assert(value[0] == 0.0f && value[1] == 0.0f);
        for (axis = 0U; axis < 2U; ++axis) {
            peak[axis] = fmaxf(peak[axis], fabsf(value[axis]));
            integral[axis] += value[axis] * 0.004f;
            if (value[axis] > 0.5f) ++positive[axis];
            if (value[axis] < -0.5f) ++negative[axis];
        }
    }
    assert(peak[0] > 29.0f && peak[0] <= YAW_IDENT_SPEED_BIG_PEAK_DPS);
    assert(peak[1] > 58.0f && peak[1] <= YAW_IDENT_SPEED_SMALL_PEAK_DPS);
    assert(fabsf(integral[0]) < 0.02f && fabsf(integral[1]) < 0.02f);
    assert(positive[0] > 1000U && negative[0] > 1000U);
    assert(positive[1] > 1000U && negative[1] > 1000U);
}

static void speed_profile_keeps_only_small_relative_travel(void)
{
    YawIdent_State s = {0};
    YawIdent_Request r = {1U, 0.0f, 5U, YAW_IDENT_PROFILE_SPEED, 0U};
    YawIdent_Observation o = observation(100U);
    assert(YawIdent_Arm(&s, &r, &o));
    o.remote_up = 0U; o.remote_down = 1U; o.now_ms += 4U;
    assert(YawIdent_Step(&s, &o).reference_valid);
    o.big_deg = 179.0f;
    o.imu_yaw_deg = 1080.0f;
    o.small_joint_deg = 19.9f;
    o.now_ms += 4U;
    assert(YawIdent_Step(&s, &o).reference_valid);
    o.small_joint_deg = 20.1f;
    o.now_ms += 4U;
    assert(!YawIdent_Step(&s, &o).reference_valid);
    assert(s.phase == YAW_IDENT_ABORTED && s.reason == YAW_IDENT_TRAVEL);
}

static void relaxed_center(void)
{
    unsigned i;
    for (i = 0U; i < 4U; ++i) {
        YawIdent_State s = {0};
        YawIdent_Request r = {1U, 0.0f, 0U, YAW_IDENT_PROFILE_BENCH, 0U};
        YawIdent_Observation o = observation(100U);
        o.small_joint_deg = i % 2U ? -10.0f : 10.0f;
        if (i >= 2U) o.small_joint_deg *= 1.001f;
        assert(YawIdent_Arm(&s, &r, &o));
        {
            YawIdent_Result result = YawIdent_Step(&s, &o);
            assert(result.phase == YAW_IDENT_BASELINE && !result.reference_valid);
        }
    }
    for (i = 0U; i < 3U; ++i) {
        YawIdent_State s = {0};
        YawIdent_Request r = {1U, 15.0f, 1U, YAW_IDENT_PROFILE_SLOW, 0U};
        YawIdent_Observation o = observation(100U);
        o.small_joint_deg = i == 0U ? -10.0f : (i == 1U ? 4.0f : 5.0f);
        assert(YawIdent_Arm(&s, &r, &o));
        o.small_joint_deg = 5.0f;
        o.remote_up = 0U; o.remote_down = 1U;
        assert(YawIdent_Step(&s, &o).reference_valid);
    }
}

static void wire(void)
{
    YawIdent_Packet p = {1U, 1500U, YAW_IDENT_ARM_SLOW, 1U, YAW_IDENT_MAGIC}, decoded;
    assert(YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.axis = 2U; assert(!YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.amplitude_cdeg = 1000U; p.op = YAW_IDENT_ARM_SLOW_REVERSE;
    assert(YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.op = YAW_IDENT_ARM; assert(!YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.axis = 0U; p.amplitude_cdeg = 0U; p.op = YAW_IDENT_BENCH;
    assert(YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.op = YAW_IDENT_STREAM_ACK; assert(!YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.amplitude_cdeg = 9001U; assert(YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.amplitude_cdeg = 9002U; assert(!YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.op = YAW_IDENT_ARM_DUAL_A; p.axis = 3U; p.amplitude_cdeg = 0U;
    assert(YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.op = YAW_IDENT_ARM_DUAL_B; assert(YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.axis = 2U; assert(!YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.axis = 3U; p.amplitude_cdeg = 1U; assert(!YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.op = YAW_IDENT_ARM_CD_C; p.axis = 4U; p.amplitude_cdeg = 0U;
    assert(YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.op = YAW_IDENT_ARM_CD_D; assert(YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.op = YAW_IDENT_ARM_CD_E; assert(YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.axis = 3U; assert(!YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.op = YAW_IDENT_ARM_SPEED_S1; p.axis = 5U;
    assert(YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.op = YAW_IDENT_ARM_SPEED_S2; assert(YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.op = YAW_IDENT_ARM_SPEED_S3; assert(YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
    p.axis = 4U; assert(!YawIdent_DecodeRequest((uint8_t *)&p, &decoded));
}

int main(void)
{
    full_profile(1U, 0U, 0U); full_profile(1U, 1U, 0U);
    full_profile(2U, 0U, 0U); full_profile(2U, 1U, 0U); full_profile(0U, 0U, 1U);
    dual_profile(0U); dual_profile(1U);
    cd_profile(0U); cd_profile(1U); cd_profile(2U);
    speed_profile(0U); speed_profile(1U); speed_profile(2U);
    speed_profile_keeps_only_small_relative_travel();
    bounds_and_bench(); relaxed_center(); wire();
    puts("Slow waveform, wire bounds, wraparound and zero-reference bench tests passed.");
    return 0;
}
