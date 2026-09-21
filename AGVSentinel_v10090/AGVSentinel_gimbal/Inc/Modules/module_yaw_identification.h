#ifndef MODULE_YAW_IDENTIFICATION_H
#define MODULE_YAW_IDENTIFICATION_H

#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ != 0)
#error "Yaw identification requires finite-value checks: use AC6 -ffp-mode=full"
#endif

/* Pure scheduler; app_yaw_identification owns I/O, arbitration and shutdown. */
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "module_yaw_limits.h"

#define YAW_IDENT_ARM_TIMEOUT_MS 15000U
#define YAW_IDENT_HEARTBEAT_MS 300U
#define YAW_IDENT_MAX_STEP_MS 10U
#define YAW_IDENT_PRE_MS 500U
#define YAW_IDENT_DRIVE_MS 3000U
#define YAW_IDENT_POST_MS 500U
#define YAW_IDENT_DURATION_MS (YAW_IDENT_PRE_MS + YAW_IDENT_DRIVE_MS + YAW_IDENT_POST_MS)
#define YAW_IDENT_PERIOD_MS 4U
#define YAW_IDENT_SAMPLES (YAW_IDENT_DURATION_MS / YAW_IDENT_PERIOD_MS + 1U)
#define YAW_IDENT_MAX_OFFSET_DEG 5.0f
#define YAW_IDENT_TRAVEL_DEG 12.0f
#define YAW_IDENT_BIG_RATE_ABORT_DPS 360.0f
#define YAW_IDENT_SMALL_RATE_ABORT_DPS 180.0f
#define YAW_IDENT_SLOW_PRE_MS 2000U
#define YAW_IDENT_SLOW_DRIVE_MS 14000U
#define YAW_IDENT_SLOW_POST_MS 4000U
#define YAW_IDENT_SLOW_DURATION_MS 20000U
#define YAW_IDENT_SLOW_SAMPLES (YAW_IDENT_SLOW_DURATION_MS / YAW_IDENT_PERIOD_MS + 1U)
#define YAW_IDENT_SLOW_BIG_PEAK_DEG 15.0f
#define YAW_IDENT_SLOW_SMALL_PEAK_DEG 10.0f
#define YAW_IDENT_SLOW_CENTER_DEG 10.0f
#define YAW_IDENT_SLOW_BIG_TRAVEL_DEG 25.0f
#define YAW_IDENT_SLOW_SMALL_TRAVEL_DEG 20.0f
#define YAW_IDENT_SLOW_HEADING_TRAVEL_DEG 20.0f
#define YAW_IDENT_DUAL_BIG_PEAK_DEG 3.0f
#define YAW_IDENT_DUAL_SMALL_PEAK_DEG 2.0f
#define YAW_IDENT_CD_PRE_MS 3000U
#define YAW_IDENT_CD_REVERSAL_MS 6000U
#define YAW_IDENT_CD_MULTISINE_MS 22000U
#define YAW_IDENT_CD_DRIVE_MS (YAW_IDENT_CD_REVERSAL_MS + YAW_IDENT_CD_MULTISINE_MS)
#define YAW_IDENT_CD_POST_MS 5000U
#define YAW_IDENT_CD_DURATION_MS (YAW_IDENT_CD_PRE_MS + YAW_IDENT_CD_DRIVE_MS + YAW_IDENT_CD_POST_MS)
#define YAW_IDENT_CD_SAMPLES (YAW_IDENT_CD_DURATION_MS / YAW_IDENT_PERIOD_MS + 1U)
#define YAW_IDENT_SPEED_PRE_MS 3000U
#define YAW_IDENT_SPEED_DRIVE_MS 28000U
#define YAW_IDENT_SPEED_POST_MS 5000U
#define YAW_IDENT_SPEED_DURATION_MS (YAW_IDENT_SPEED_PRE_MS + YAW_IDENT_SPEED_DRIVE_MS + YAW_IDENT_SPEED_POST_MS)
#define YAW_IDENT_SPEED_SAMPLES (YAW_IDENT_SPEED_DURATION_MS / YAW_IDENT_PERIOD_MS + 1U)
#define YAW_IDENT_SPEED_BIG_PEAK_DPS 30.0f
#define YAW_IDENT_SPEED_SMALL_PEAK_DPS 60.0f
#define YAW_IDENT_STREAM_TIMEOUT_MS 300U

enum { YAW_IDENT_PROFILE_SHORT = 0, YAW_IDENT_PROFILE_SLOW = 1,
       YAW_IDENT_PROFILE_BENCH = 2, YAW_IDENT_PROFILE_DUAL = 3,
       YAW_IDENT_PROFILE_CD = 4, YAW_IDENT_PROFILE_SPEED = 5 };

typedef enum {
    YAW_IDENT_IDLE = 0,
    YAW_IDENT_ARMED,
    YAW_IDENT_BASELINE,
    YAW_IDENT_EXCITE,
    YAW_IDENT_SETTLE,
    YAW_IDENT_DONE,
    YAW_IDENT_ABORTED
} YawIdent_Phase;

typedef enum {
    YAW_IDENT_OK = 0,
    YAW_IDENT_OPERATOR_STOP,
    YAW_IDENT_LINK_LOST,
    YAW_IDENT_FEEDBACK_BAD,
    YAW_IDENT_TRAVEL,
    YAW_IDENT_TIMING,
    YAW_IDENT_ARM_EXPIRED,
    YAW_IDENT_CAPTURE_NOT_READY,
    YAW_IDENT_CONFIG_CHANGED,
    YAW_IDENT_CONTROL_FAILED,
    YAW_IDENT_STREAM_STALLED,
    YAW_IDENT_BUFFER_FULL,
    YAW_IDENT_YAW_CONTROL_INACTIVE,
    YAW_IDENT_OUTPUT_NONFINITE,
    YAW_IDENT_PITCH_OFFLINE
} YawIdent_Reason;

typedef struct {
    uint32_t now_ms;
    float big_deg;
    float small_joint_deg;
    float imu_yaw_deg;
    float big_rate_dps;
    float small_rate_dps;
    uint16_t imu_age_ms, big_age_ms, small_age_ms, remote_age_ms;
    uint16_t capture_capacity;
    uint16_t feedback_faults; /* Record flag bits 12..15: INS, big, small, invalid. */
    uint8_t feedback_valid;
    uint8_t remote_up;
    uint8_t remote_down;
    uint8_t remote_middle;
    uint8_t operator_neutral;
    uint8_t yaw_outputs_off;
    uint8_t capture_ready;
} YawIdent_Observation;

typedef struct {
    uint32_t trial_id;
    float amplitude_deg;
    uint8_t axis; /* 1: big joint position; 2: small inertial heading. */
    uint8_t profile;
    uint8_t reverse;
} YawIdent_Request;

typedef struct {
    uint32_t last_id, armed_ms, heartbeat_ms, previous_ms, started_ms;
    YawIdent_Request request;
    YawIdent_Phase phase;
    YawIdent_Reason reason;
    uint8_t trigger_released;
    float start_big_deg, start_small_deg, start_yaw_deg;
} YawIdent_State;

typedef struct {
    float big_offset_deg;
    float small_heading_offset_deg;
    float big_anchor_deg;
    float small_heading_anchor_deg;
    uint32_t elapsed_ms;
    YawIdent_Phase phase;
    YawIdent_Reason reason;
    uint8_t reference_valid;
} YawIdent_Result;

static inline uint32_t YawIdent_Duration(const YawIdent_Request *r)
{
    if (r->profile == YAW_IDENT_PROFILE_SPEED) return YAW_IDENT_SPEED_DURATION_MS;
    if (r->profile == YAW_IDENT_PROFILE_CD) return YAW_IDENT_CD_DURATION_MS;
    return r->profile ? YAW_IDENT_SLOW_DURATION_MS : YAW_IDENT_DURATION_MS;
}

static inline uint16_t YawIdent_Samples(const YawIdent_Request *r)
{
    return (uint16_t)(r->profile == YAW_IDENT_PROFILE_SPEED ? YAW_IDENT_SPEED_SAMPLES :
        (r->profile == YAW_IDENT_PROFILE_CD ?
        YAW_IDENT_CD_SAMPLES : (r->profile ? YAW_IDENT_SLOW_SAMPLES : YAW_IDENT_SAMPLES)));
}

static inline uint32_t YawIdent_PreDuration(const YawIdent_Request *r)
{
    return r->profile == YAW_IDENT_PROFILE_SPEED ? YAW_IDENT_SPEED_PRE_MS :
        (r->profile == YAW_IDENT_PROFILE_CD ? YAW_IDENT_CD_PRE_MS :
        (r->profile ? YAW_IDENT_SLOW_PRE_MS : YAW_IDENT_PRE_MS));
}

static inline uint32_t YawIdent_DriveDuration(const YawIdent_Request *r)
{
    return r->profile == YAW_IDENT_PROFILE_SPEED ? YAW_IDENT_SPEED_DRIVE_MS :
        (r->profile == YAW_IDENT_PROFILE_CD ? YAW_IDENT_CD_DRIVE_MS :
        (r->profile ? YAW_IDENT_SLOW_DRIVE_MS : YAW_IDENT_DRIVE_MS));
}

static inline float YawIdent_Center(const YawIdent_Request *r)
{
    return r->profile ? YAW_IDENT_SLOW_CENTER_DEG : 10.0f;
}

static inline uint8_t YawIdent_RelativeTravel(const YawIdent_State *s,
    const YawIdent_Observation *o)
{
    /* The cable-free speed-reference profile has no mechanical big-axis or
     * inertial-heading travel stop.  The small joint remains constrained by
     * both this relative bound and YawIdent_InsideTravel's absolute limits. */
    if (s->request.profile == YAW_IDENT_PROFILE_SPEED)
        return (uint8_t)(fabsf(o->small_joint_deg - s->start_small_deg) <=
                         YAW_IDENT_SLOW_SMALL_TRAVEL_DEG);
    return (uint8_t)(fabsf(remainderf(o->big_deg - s->start_big_deg, 360.0f)) <=
        (s->request.profile ? YAW_IDENT_SLOW_BIG_TRAVEL_DEG : YAW_IDENT_TRAVEL_DEG) &&
        fabsf(o->small_joint_deg - s->start_small_deg) <=
        (s->request.profile ? YAW_IDENT_SLOW_SMALL_TRAVEL_DEG : YAW_IDENT_TRAVEL_DEG) &&
        fabsf(o->imu_yaw_deg - s->start_yaw_deg) <=
        (s->request.profile ? YAW_IDENT_SLOW_HEADING_TRAVEL_DEG : YAW_IDENT_TRAVEL_DEG));
}

static inline uint8_t YawIdent_RequestValid(const YawIdent_Request *r)
{
    if (r->profile > YAW_IDENT_PROFILE_SPEED || !isfinite(r->amplitude_deg) ||
        (r->profile == YAW_IDENT_PROFILE_CD || r->profile == YAW_IDENT_PROFILE_SPEED ?
         r->reverse > 2U : r->reverse > 1U)) return 0U;
    if (r->profile == YAW_IDENT_PROFILE_BENCH)
        return (uint8_t)(r->axis == 0U && r->amplitude_deg == 0.0f && !r->reverse);
    if (r->profile == YAW_IDENT_PROFILE_DUAL)
        return (uint8_t)(r->axis == 3U && r->amplitude_deg == 0.0f);
    if (r->profile == YAW_IDENT_PROFILE_CD)
        return (uint8_t)(r->axis == 4U && r->amplitude_deg == 0.0f);
    if (r->profile == YAW_IDENT_PROFILE_SPEED)
        return (uint8_t)(r->axis == 5U && r->amplitude_deg == 0.0f);
    return (uint8_t)((r->axis == 1U || r->axis == 2U) && r->amplitude_deg > 0.0f &&
        r->amplitude_deg <= (r->profile == YAW_IDENT_PROFILE_SLOW ?
            (r->axis == 1U ? YAW_IDENT_SLOW_BIG_PEAK_DEG : YAW_IDENT_SLOW_SMALL_PEAK_DEG) :
            YAW_IDENT_MAX_OFFSET_DEG) && (r->profile || !r->reverse));
}

static inline void YawIdent_Init(YawIdent_State *s)
{
    /* Boot initialization only. Reinitializing must not defeat replay protection. */
    memset(s, 0, sizeof(*s));
}

static inline uint8_t YawIdent_FeedbackValid(const YawIdent_Observation *o)
{
    return (uint8_t)(o->feedback_valid && o->imu_age_ms <= 10U &&
        o->big_age_ms <= 20U && o->small_age_ms <= 20U &&
        isfinite(o->big_deg) && o->big_deg >= 0.0f && o->big_deg < 360.0f &&
        isfinite(o->small_joint_deg) && isfinite(o->imu_yaw_deg) &&
        isfinite(o->big_rate_dps) && isfinite(o->small_rate_dps));
}

static inline uint8_t YawIdent_InsideTravel(const YawIdent_Observation *o)
{
    return (uint8_t)(o->small_joint_deg >= YAW_LIMIT_MIN_DEG + 5.0f &&
        o->small_joint_deg <= YAW_LIMIT_MAX_DEG - 5.0f &&
        fabsf(o->big_rate_dps) <= YAW_IDENT_BIG_RATE_ABORT_DPS &&
        fabsf(o->small_rate_dps) <= YAW_IDENT_SMALL_RATE_ABORT_DPS);
}

static inline uint8_t YawIdent_StartClearance(const YawIdent_Request *r,
    const YawIdent_Observation *o)
{
    /* The zero-output bench does not need a reserved motion envelope. */
    return (uint8_t)((r->profile != YAW_IDENT_PROFILE_SLOW &&
                      r->profile != YAW_IDENT_PROFILE_DUAL &&
                      r->profile != YAW_IDENT_PROFILE_CD &&
                      r->profile != YAW_IDENT_PROFILE_SPEED) ||
        (o->small_joint_deg - YAW_IDENT_SLOW_SMALL_TRAVEL_DEG >= YAW_LIMIT_MIN_DEG + 5.0f &&
         o->small_joint_deg + YAW_IDENT_SLOW_SMALL_TRAVEL_DEG <= YAW_LIMIT_MAX_DEG - 5.0f));
}

static inline uint8_t YawIdent_Arm(YawIdent_State *s,
    const YawIdent_Request *request, const YawIdent_Observation *o)
{
    /* Operator-observable setup is intentionally not an ARM admission gate.
     * Runtime supervision still checks remote, feedback, travel and rate before
     * and throughout reference application. */
    if ((s->phase != YAW_IDENT_IDLE && s->phase != YAW_IDENT_DONE &&
         s->phase != YAW_IDENT_ABORTED) || request->trial_id == 0U ||
        request->trial_id <= s->last_id ||
        !YawIdent_RequestValid(request) ||
        !o->yaw_outputs_off || !o->capture_ready ||
        o->capture_capacity < YAW_IDENT_SAMPLES)
        return 0U;
    s->request = *request;
    s->last_id = request->trial_id;
    s->armed_ms = s->heartbeat_ms = s->previous_ms = o->now_ms;
    s->trigger_released = (uint8_t)(request->profile == YAW_IDENT_PROFILE_BENCH || !o->remote_down);
    s->phase = YAW_IDENT_ARMED;
    s->reason = YAW_IDENT_OK;
    return 1U;
}

static inline uint8_t YawIdent_Heartbeat(YawIdent_State *s, uint32_t trial_id,
    uint32_t now_ms)
{
    if (s->phase < YAW_IDENT_ARMED || s->phase > YAW_IDENT_SETTLE ||
        trial_id != s->request.trial_id ||
        (uint32_t)(now_ms - s->heartbeat_ms) > YAW_IDENT_HEARTBEAT_MS)
        return 0U;
    s->heartbeat_ms = now_ms;
    return 1U;
}

static inline void YawIdent_Abort(YawIdent_State *s, YawIdent_Reason reason)
{
    s->phase = YAW_IDENT_ABORTED;
    s->reason = reason;
}

static inline float YawIdent_Wave(uint32_t drive_ms, float amplitude)
{
    const float pi = 3.14159265358979323846f;
    float t, window;
    if (drive_ms == 0U || drive_ms >= YAW_IDENT_DRIVE_MS) return 0.0f;
    t = drive_ms * 0.001f;
    window = sinf(pi * t / (YAW_IDENT_DRIVE_MS * 0.001f));
    return amplitude * window * window *
        (0.5f * sinf(2.0f * pi * t) +
         0.3f * sinf(4.0f * pi * t) + 0.2f * sinf(8.0f * pi * t));
}

static inline float YawIdent_Smooth(float x)
{
    return x * x * x * (10.0f + x * (-15.0f + 6.0f * x));
}

/* Drive-relative times: ramp 2s, dwell 3s, reverse 4s, dwell 3s, return 2s. */
static inline float YawIdent_SlowWave(uint32_t drive_ms, float amplitude)
{
    if (drive_ms < 2000U) return amplitude * YawIdent_Smooth(drive_ms / 2000.0f);
    if (drive_ms < 5000U) return amplitude;
    if (drive_ms < 9000U)
        return amplitude * (1.0f - 2.0f * YawIdent_Smooth((drive_ms - 5000U) / 4000.0f));
    if (drive_ms < 12000U) return -amplitude;
    if (drive_ms < 14000U)
        return amplitude * (YawIdent_Smooth((drive_ms - 12000U) / 2000.0f) - 1.0f);
    return 0.0f;
}

/* Two disjoint harmonic sets over the 14s drive interval. phase_set 0=A, 1=B. */
static inline float YawIdent_DualWave(uint32_t drive_ms, uint8_t axis, uint8_t phase_set)
{
    const float pi = 3.14159265358979323846f;
    static const uint8_t harmonic[2][3] = {{2U, 8U, 14U}, {5U, 11U, 17U}};
    static const float amplitude[2][3] = {{1.5f, 0.9f, 0.6f}, {1.0f, 0.6f, 0.4f}};
    static const float phase[2][2][3] = {
        {{0.2f, 1.1f, 2.2f}, {0.7f, 1.7f, 2.8f}},
        {{1.3f, 2.4f, 0.4f}, {2.0f, 0.5f, 1.4f}}
    };
    float t, window, result = 0.0f;
    unsigned i;
    if (drive_ms == 0U || drive_ms >= YAW_IDENT_SLOW_DRIVE_MS || axis > 1U || phase_set > 1U)
        return 0.0f;
    t = drive_ms * 0.001f;
    window = sinf(pi * t / (YAW_IDENT_SLOW_DRIVE_MS * 0.001f));
    window *= window;
    for (i = 0U; i < 3U; ++i)
        result += amplitude[axis][i] * sinf(2.0f * pi * harmonic[axis][i] * t /
            (YAW_IDENT_SLOW_DRIVE_MS * 0.001f) + phase[phase_set][axis][i]);
    return window * result;
}

/* Cable-free ladder: the exact former E shape at 1x, 4x or 5x.  Keeping the
 * shape fixed makes R1 a mechanical-change comparison; F4/F5 expose amplitude
 * nonlinearity without changing frequency content. phase_set 0=R1, 1=F4,
 * 2=F5. Values are frozen in cable_free_amplitude_ladder_plan.json. */
static inline float YawIdent_CDWave(uint32_t drive_ms, uint8_t axis, uint8_t phase_set)
{
    const float pi = 3.14159265358979323846f;
    static const float scale[3] = {1.0f, 4.0f, 5.0f};
    static const float levels[2][12] = {
        {0.0f, 0.35f, 0.0f, -0.55f, -0.55f, 0.0f, -0.35f, 0.0f, 0.55f, 0.55f, 0.0f, 0.0f},
        {0.0f, -0.40f, -0.40f, 0.0f, 0.25f, 0.0f, -0.25f, 0.0f, 0.40f, 0.40f, 0.0f, 0.0f}
    };
    static const uint8_t harmonic[2][5] = {
        {4U, 11U, 25U, 49U, 73U}, {7U, 16U, 34U, 61U, 89U}
    };
    static const float amplitude[2][5] = {
        {1.45f, 0.86f, 0.45f, 0.18f, 0.06f},
        {0.97f, 0.605f, 0.28f, 0.11f, 0.035f}
    };
    static const float phase[2][5] = {
        {1.16f, 3.93f, 1.60f, 4.17f, 1.61f},
        {3.50f, 2.54f, 1.04f, 5.55f, 5.02f}
    };
    float t, window, result = 0.0f;
    unsigned i;
    if (axis > 1U || phase_set > 2U || drive_ms >= YAW_IDENT_CD_DRIVE_MS) return 0.0f;
    if (drive_ms < YAW_IDENT_CD_REVERSAL_MS) {
        unsigned index = drive_ms / 500U;
        unsigned previous = index ? index - 1U : 0U;
        uint32_t within = drive_ms - index * 500U;
        float blend = 1.0f;
        if (within < 200U)
            blend = 0.5f - 0.5f * cosf(pi * within / 200.0f);
        return scale[phase_set] * (levels[axis][previous] +
            (levels[axis][index] - levels[axis][previous]) * blend);
    }
    drive_ms -= YAW_IDENT_CD_REVERSAL_MS;
    if (drive_ms == 0U || drive_ms >= YAW_IDENT_CD_MULTISINE_MS) return 0.0f;
    t = drive_ms * 0.001f;
    window = sinf(pi * t / (YAW_IDENT_CD_MULTISINE_MS * 0.001f));
    window *= window;
    for (i = 0U; i < 5U; ++i)
        result += amplitude[axis][i] * sinf(2.0f * pi * harmonic[axis][i] * t /
            (YAW_IDENT_CD_MULTISINE_MS * 0.001f) + phase[axis][i]);
    return scale[phase_set] * window * result;
}

/* Direct speed references for identifying the retained inner speed loops.
 * They are analytic derivatives of windowed position primitives, so each
 * drive starts/ends at zero speed and has exactly zero net reference travel.
 * S1/S2/S3 use globally disjoint harmonics. S3 is the prospective validation
 * set frozen after S2 failed only its raw-record-count data gate.
 * Returned units are degrees/second. */
static inline float YawIdent_SpeedWave(uint32_t drive_ms, uint8_t axis, uint8_t phase_set)
{
    const float pi = 3.14159265358979323846f;
    const float duration_s = YAW_IDENT_SPEED_DRIVE_MS * 0.001f;
    static const uint8_t harmonic[3][2][4] = {
        {{5U, 13U, 29U, 61U}, {8U, 19U, 41U, 83U}},
        {{7U, 17U, 37U, 79U}, {11U, 23U, 47U, 101U}},
        {{9U, 21U, 43U, 73U}, {10U, 27U, 53U, 91U}}
    };
    static const float primitive_amplitude[3][2][4] = {
        {{4.4576823f, 3.3432617f, 1.6716309f, 0.5572103f},
         {5.6957129f, 4.5565703f, 2.2782852f, 0.9113141f}},
        {{3.2938282f, 2.4703712f, 1.2351856f, 0.4117285f},
         {4.5531931f, 3.6425546f, 1.8212773f, 0.7285109f}},
        {{2.9025746f, 2.2617464f, 1.2150312f, 0.3903836f},
         {5.1704974f, 3.4818164f, 1.9511311f, 0.6714932f}}
    };
    static const float phase[3][2][4] = {
        {{0.31f, 1.47f, 2.71f, 4.19f}, {1.03f, 2.33f, 4.41f, 0.57f}},
        {{2.02f, 0.63f, 3.82f, 5.21f}, {0.49f, 3.07f, 1.58f, 4.76f}},
        {{0.9650392f, 4.5401046f, 2.7112651f, 1.3533682f},
         {0.4001477f, 4.6934328f, 6.2240131f, 0.1600084f}}
    };
    float t, x, window, window_rate, position = 0.0f, position_rate = 0.0f;
    float result, limit;
    unsigned i;
    if (drive_ms == 0U || drive_ms >= YAW_IDENT_SPEED_DRIVE_MS ||
        axis > 1U || phase_set > 2U) return 0.0f;
    t = drive_ms * 0.001f;
    x = t / duration_s;
    window = sinf(pi * x);
    window *= window;
    window_rate = (pi / duration_s) * sinf(2.0f * pi * x);
    for (i = 0U; i < 4U; ++i) {
        float omega = 2.0f * pi * harmonic[phase_set][axis][i] / duration_s;
        float angle = omega * t + phase[phase_set][axis][i];
        position += primitive_amplitude[phase_set][axis][i] * sinf(angle);
        position_rate += primitive_amplitude[phase_set][axis][i] * omega * cosf(angle);
    }
    result = window_rate * position + window * position_rate;
    limit = axis ? YAW_IDENT_SPEED_SMALL_PEAK_DPS : YAW_IDENT_SPEED_BIG_PEAK_DPS;
    return result > limit ? limit : (result < -limit ? -limit : result);
}

static inline YawIdent_Result YawIdent_Step(YawIdent_State *s,
    const YawIdent_Observation *o)
{
    YawIdent_Result result = {0};
    uint32_t elapsed;
    if (s->phase < YAW_IDENT_ARMED || s->phase > YAW_IDENT_SETTLE) goto finish;
    if ((uint32_t)(o->now_ms - s->previous_ms) > YAW_IDENT_MAX_STEP_MS) {
        YawIdent_Abort(s, YAW_IDENT_TIMING); goto finish;
    }
    s->previous_ms = o->now_ms;
    if ((uint32_t)(o->now_ms - s->heartbeat_ms) > YAW_IDENT_HEARTBEAT_MS) {
        YawIdent_Abort(s, YAW_IDENT_LINK_LOST); goto finish;
    }
    if (o->remote_age_ms > 50U || !o->operator_neutral ||
        ((unsigned)o->remote_up + o->remote_down + o->remote_middle != 1U)) {
        YawIdent_Abort(s, YAW_IDENT_OPERATOR_STOP); goto finish;
    }
    if (!YawIdent_FeedbackValid(o)) {
        YawIdent_Abort(s, YAW_IDENT_FEEDBACK_BAD); goto finish;
    }
    if (!YawIdent_InsideTravel(o)) {
        YawIdent_Abort(s, YAW_IDENT_TRAVEL); goto finish;
    }
    if (s->phase == YAW_IDENT_ARMED) {
        if ((uint32_t)(o->now_ms - s->armed_ms) > YAW_IDENT_ARM_TIMEOUT_MS) {
            YawIdent_Abort(s, YAW_IDENT_ARM_EXPIRED); goto finish;
        }
        if (!o->capture_ready || o->capture_capacity < YAW_IDENT_SAMPLES) {
            YawIdent_Abort(s, YAW_IDENT_CAPTURE_NOT_READY); goto finish;
        }
        /* A three-position switch passes through middle before reaching DOWN.
         * Only the waiting phase permits middle, with all outputs still off. */
        if (!(s->request.profile == YAW_IDENT_PROFILE_BENCH ? o->remote_up : o->remote_down)) {
            if (s->request.profile == YAW_IDENT_PROFILE_BENCH)
                YawIdent_Abort(s, YAW_IDENT_OPERATOR_STOP);
            else s->trigger_released = 1U;
            if (!o->yaw_outputs_off) YawIdent_Abort(s, YAW_IDENT_OPERATOR_STOP);
            goto finish;
        }
        if (s->request.profile != YAW_IDENT_PROFILE_BENCH && !s->trigger_released)
            goto finish;
        s->started_ms = o->now_ms;
        s->start_big_deg = o->big_deg;
        s->start_small_deg = o->small_joint_deg;
        s->start_yaw_deg = o->imu_yaw_deg;
        s->phase = YAW_IDENT_BASELINE;
    }
    if (s->request.profile == YAW_IDENT_PROFILE_BENCH ?
        (!o->remote_up || !o->yaw_outputs_off) : (!o->remote_down || o->remote_up)) {
        YawIdent_Abort(s, YAW_IDENT_OPERATOR_STOP); goto finish;
    }
    if (!o->capture_ready || o->capture_capacity < YAW_IDENT_SAMPLES) {
        YawIdent_Abort(s, YAW_IDENT_CAPTURE_NOT_READY); goto finish;
    }
    if (!YawIdent_RelativeTravel(s, o)) {
        YawIdent_Abort(s, YAW_IDENT_TRAVEL); goto finish;
    }
    elapsed = o->now_ms - s->started_ms;
    result.elapsed_ms = elapsed;
    result.big_anchor_deg = s->start_big_deg;
    result.small_heading_anchor_deg = s->start_yaw_deg;
    if (elapsed >= YawIdent_Duration(&s->request)) {
        s->phase = YAW_IDENT_DONE;
        goto finish;
    }
    result.reference_valid = (uint8_t)(s->request.profile != YAW_IDENT_PROFILE_BENCH);
    if (elapsed < YawIdent_PreDuration(&s->request))
        s->phase = YAW_IDENT_BASELINE;
    else if (elapsed < YawIdent_PreDuration(&s->request) + YawIdent_DriveDuration(&s->request)) {
        uint32_t drive_ms = elapsed - YawIdent_PreDuration(&s->request);
        float offset = s->request.profile ?
            YawIdent_SlowWave(drive_ms,
                s->request.reverse ? -s->request.amplitude_deg : s->request.amplitude_deg) :
            YawIdent_Wave(drive_ms, s->request.amplitude_deg);
        s->phase = YAW_IDENT_EXCITE;
        if (s->request.profile == YAW_IDENT_PROFILE_SPEED) {
            result.big_offset_deg = YawIdent_SpeedWave(drive_ms, 0U, s->request.reverse);
            result.small_heading_offset_deg = YawIdent_SpeedWave(drive_ms, 1U, s->request.reverse);
        } else if (s->request.profile == YAW_IDENT_PROFILE_CD) {
            result.big_offset_deg = YawIdent_CDWave(drive_ms, 0U, s->request.reverse);
            result.small_heading_offset_deg = YawIdent_CDWave(drive_ms, 1U, s->request.reverse);
        } else if (s->request.profile == YAW_IDENT_PROFILE_DUAL) {
            result.big_offset_deg = YawIdent_DualWave(drive_ms, 0U, s->request.reverse);
            result.small_heading_offset_deg = YawIdent_DualWave(drive_ms, 1U, s->request.reverse);
        } else if (s->request.axis == 1U) result.big_offset_deg = offset;
        else result.small_heading_offset_deg = offset;
    } else s->phase = YAW_IDENT_SETTLE;
finish:
    result.phase = s->phase;
    result.reason = s->reason;
    return result;
}

#endif
