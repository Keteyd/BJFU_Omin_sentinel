#ifndef MODULE_YAW_PEAKS_H
#define MODULE_YAW_PEAKS_H

#include <math.h>
#include <stdint.h>
#include <string.h>

/* Diagnostic only: task samples, not a count of distinct CAN frames. */
typedef struct {
    uint32_t start_ms, last_ms;
    int32_t position, minimum, maximum;
    uint16_t samples, span, error, speed, effort, previous;
    uint8_t started, tracking, invalid, saturated;
} YawPeaks_State;

static inline void YawPeaks_Reset(YawPeaks_State *s)
{
    memset(s, 0, sizeof(*s));
}

static inline uint16_t YawPeaks_Scale(YawPeaks_State *s, float value, float scale)
{
    float v = fabsf(value) * scale;
    if (!isfinite(v) || v > 65535.0f) {
        s->invalid = 1U;
        return 65535U;
    }
    return (uint16_t)(v + 0.5f);
}

static inline void YawPeaks_Update(YawPeaks_State *s, uint32_t now,
    uint8_t active, uint8_t fresh, uint16_t raw, float error,
    float speed, float effort, uint8_t saturated)
{
    uint16_t v;
    if (!s->started) {
        s->started = 1U;
        s->start_ms = now;
    } else if (now - s->last_ms > 10U) {
        s->invalid = 1U;
        s->tracking = 0U;
    }
    s->last_ms = now;
    if (!fresh || raw >= 8192U || !isfinite(error) ||
        !isfinite(speed) || !isfinite(effort)) {
        s->invalid = 1U;
        s->tracking = 0U;
        return;
    }
    if (!active) {
        s->tracking = 0U;
        return;
    }
    if (s->samples == 0x3FFFU) {
        s->invalid = 1U;
        return;
    }
    ++s->samples;
    s->saturated |= (uint8_t)(saturated != 0U);
    if (!s->tracking) {
        s->position = s->minimum = s->maximum = 0;
        s->tracking = 1U;
    } else {
        int32_t delta = (int32_t)raw - s->previous;
        if (delta > 4096) delta -= 8192;
        if (delta < -4096) delta += 8192;
        if (delta == 4096 || delta == -4096) s->invalid = 1U;
        s->position += delta;
        if (s->position < s->minimum) s->minimum = s->position;
        if (s->position > s->maximum) s->maximum = s->position;
        if (s->maximum - s->minimum > 65535) {
            s->span = 65535U;
            s->invalid = 1U;
            s->tracking = 0U;
        } else if (s->maximum - s->minimum > s->span) {
            s->span = (uint16_t)(s->maximum - s->minimum);
        }
    }
    s->previous = raw;
    v = YawPeaks_Scale(s, error, 100.0f);
    if (v > s->error) s->error = v;
    v = YawPeaks_Scale(s, speed, 100.0f);
    if (v > s->speed) s->speed = v;
    v = YawPeaks_Scale(s, effort, 1000.0f);
    if (v > s->effort) s->effort = v;
}

/* Six little-endian uint16 words on STM32; retain until TX accepts the packet. */
static inline void YawPeaks_Pack(YawPeaks_State *s, uint32_t now, uint16_t out[6])
{
    uint32_t elapsed = s->started ? now - s->start_ms : 0U;
    if (elapsed > 65535U) s->invalid = 1U;
    out[0] = (uint16_t)(s->samples | (s->saturated ? 0x4000U : 0U) |
                       (s->invalid ? 0x8000U : 0U));
    out[1] = s->span;
    out[2] = s->error;
    out[3] = s->speed;
    out[4] = s->effort;
    out[5] = (uint16_t)(elapsed > 65535U ? 65535U : elapsed);
}

static inline void YawPeaks_NextWindow(YawPeaks_State *s, uint32_t now)
{
    uint16_t previous = s->previous;
    uint8_t tracking = s->tracking;
    YawPeaks_Reset(s);
    s->started = 1U;
    s->start_ms = s->last_ms = now;
    /* Preserve the boundary delta, but not SAFE movement between segments. */
    s->previous = previous;
    s->tracking = tracking;
}

#endif
