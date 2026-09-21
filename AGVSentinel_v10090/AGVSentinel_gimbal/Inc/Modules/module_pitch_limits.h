#ifndef MODULE_PITCH_LIMITS_H
#define MODULE_PITCH_LIMITS_H

#include <math.h>
#include <stdint.h>

/* Calibration is valid only when reboot offsets are integer motor revolutions. */
static inline uint8_t PitchLimits_Align(float position, float lower, float upper,
                                       float margin, float position_max,
                                       float *minimum, float *maximum)
{
    const float revolution = 6.283185307f;
    float center;
    float offset;
    if (!isfinite(position) || !isfinite(lower) || !isfinite(upper) ||
        !isfinite(margin) || !isfinite(position_max) ||
        margin < 0.0f || upper - lower <= 2.0f * margin ||
        upper - lower >= revolution || position_max <= 0.0f) return 0U;
    center = 0.5f * (lower + upper);
    offset = roundf((position - center) / revolution) * revolution;
    /* Reject a wrong zero/calibration, rather than commanding a distant stop. */
    if (position < lower + offset - 0.02f ||
        position > upper + offset + 0.02f ||
        lower + offset < -position_max || upper + offset > position_max)
        return 0U;
    *minimum = lower + offset + margin;
    *maximum = upper + offset - margin;
    return 1U;
}

#endif
