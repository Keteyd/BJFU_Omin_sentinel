#ifndef MODULE_BIG_YAW_MANUAL_H
#define MODULE_BIG_YAW_MANUAL_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#define BIG_YAW_MANUAL_TIMEOUT_MS 50U
#define BIG_YAW_MANUAL_RATE_DEADBAND_DPS 2.0f

typedef struct {
    float rate_dps;
    uint32_t tick_ms, generation;
    uint8_t enabled;
} BigYawManual_Command;

typedef struct {
    float reference_deg;
    uint32_t generation;
    uint8_t active, fault;
} BigYawManual_State;

static inline void BigYawManual_Publish(BigYawManual_Command *c, uint8_t enabled,
    float rate_dps, uint32_t now)
{
    if (c->enabled != enabled) ++c->generation;
    c->enabled = enabled;
    c->rate_dps = fabsf(rate_dps) <= BIG_YAW_MANUAL_RATE_DEADBAND_DPS ? 0.0f : rate_dps;
    c->tick_ms = now;
}

static inline uint8_t BigYawManual_Fresh(const BigYawManual_Command *c, uint32_t now)
{
    return (uint8_t)(c->enabled && isfinite(c->rate_dps) &&
        now - c->tick_ms <= BIG_YAW_MANUAL_TIMEOUT_MS);
}

static inline uint8_t BigYawManual_Step(BigYawManual_State *s,
    const BigYawManual_Command *c, uint32_t now, uint8_t permitted,
    float position_deg, float speed_rpm, float dt)
{
    if (!permitted || !BigYawManual_Fresh(c, now) || s->fault) {
        s->active = 0U;
        return 0U;
    }
    if (!isfinite(position_deg) || !isfinite(speed_rpm) || !isfinite(dt) ||
        dt < 0.0f || dt > .01f) {
        s->fault = 1U; s->active = 0U;
        return 0U;
    }
    if (!s->active || s->generation != c->generation) {
        s->reference_deg = remainderf(position_deg, 360.0f);
        s->generation = c->generation;
        s->active = 1U;
        return 1U; /* Capture the measured pose before integrating a new command. */
    }
    s->reference_deg = remainderf(s->reference_deg + c->rate_dps * dt, 360.0f);
    if (!isfinite(s->reference_deg)) {
        s->fault = 1U; s->active = 0U;
        return 0U;
    }
    return 1U;
}
#endif
