#ifndef MODULE_BIG_YAW_FULL_H
#define MODULE_BIG_YAW_FULL_H

#include <stdint.h>
#include <string.h>
#include <math.h>

/* Order: angle P, speed P, effort, tau, angle I/D, speed I/D. */
#define BIG_YAW_FULL_COUNT 8U
typedef struct {
    float values[BIG_YAW_FULL_COUNT];
    uint32_t start_ms;
    uint16_t id, finished_id;
    uint8_t next, active;
} BigYaw_FullStage;

static inline uint8_t BigYaw_FullValid(const float v[BIG_YAW_FULL_COUNT])
{
    unsigned i;
    for (i = 0; i < BIG_YAW_FULL_COUNT; ++i)
        if (!isfinite(v[i]) || v[i] < 0.0f) return 0U;
    /* Only the motor wire-format bound remains, not a gain/tau tuning cap. */
    return (uint8_t)(v[2] <= 30.0f);
}

/* 0 partial/duplicate, 1 complete, 2 unsafe, 3 malformed, 4 expired.
 * Every chunk is CRC8-verified by the PC frame parser before reaching here. */
static inline uint8_t BigYaw_FullReceive(BigYaw_FullStage *s,
    const uint8_t packet[12], uint32_t now, uint8_t safe)
{
    uint16_t id = (uint16_t)(packet[0] | ((uint16_t)packet[1] << 8));
    uint8_t part = packet[2], result = 0U;
    if (id != 0U && id == s->finished_id) return 0U;
    if (part == 0U) {
        s->active = 1U; s->id = id; s->next = 0U; s->start_ms = now;
    }
    if (!safe) result = 2U;
    else if (id == 0U || packet[3] != 0xC3U || part >= 4U ||
             !s->active || id != s->id || part != s->next) result = 3U;
    else if (now - s->start_ms > 300U) result = 4U;
    if (result) {
        s->active = 0U; s->finished_id = id;
        return result;
    }
    memcpy(&s->values[part * 2U], packet + 4, 8);
    ++s->next;
    if (s->next < 4U) return 0U;
    s->active = 0U; s->finished_id = id;
    return BigYaw_FullValid(s->values) ? 1U : 3U;
}

#endif
