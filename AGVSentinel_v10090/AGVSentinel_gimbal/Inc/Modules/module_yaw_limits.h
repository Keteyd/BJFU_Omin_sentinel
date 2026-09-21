#ifndef MODULE_YAW_LIMITS_H
#define MODULE_YAW_LIMITS_H

#include <math.h>
#include <stdint.h>
#include <float.h>

/* Optional tuning caps: -1 disables the cap; zero still requests zero speed. */
#define YAW_RATE_UNLIMITED (-1.0f)
#define YAW_RATE_UNLIMITED_WIRE UINT16_MAX

/* Direct-drive GM6020 landmarks, 2026-09-05. Positive is toward left. */
#define YAW_LIMIT_ZERO_DEG (6816.0f * 360.0f / 8192.0f)
#define YAW_LIMIT_HARD_MIN_DEG ((5458.0f - 6816.0f) * 360.0f / 8192.0f)
#define YAW_LIMIT_HARD_MAX_DEG ((7546.0f - 6816.0f) * 360.0f / 8192.0f)
#define YAW_LIMIT_MARGIN_DEG 3.0f
#define YAW_LIMIT_MIN_DEG (YAW_LIMIT_HARD_MIN_DEG + YAW_LIMIT_MARGIN_DEG)
#define YAW_LIMIT_MAX_DEG (YAW_LIMIT_HARD_MAX_DEG - YAW_LIMIT_MARGIN_DEG)
#define YAW_LIMIT_SLOW_ZONE_DEG 8.0f
#define YAW_LIMIT_BRAKE_RPM 24.0f
#define YAW_LIMIT_FEEDBACK_MAX_AGE_MS 50U

#define YAW_LIMIT_STATUS_REFERENCE 1U
#define YAW_LIMIT_STATUS_SPEED 2U
#define YAW_LIMIT_STATUS_EFFORT 4U
#define YAW_LIMIT_STATUS_OUTSIDE 8U
#define YAW_LIMIT_STATUS_INVALID 16U

static inline float YawLimits_Clamp(float x, float lo, float hi)
{
    return fminf(fmaxf(x, lo), hi);
}

static inline float YawLimits_RateCap(float setting)
{
    if (setting == YAW_RATE_UNLIMITED) return FLT_MAX;
    return isfinite(setting) && setting >= 0.0f ? setting : 0.0f;
}

static inline float YawLimits_DecodeRateCap(uint16_t value, float scale)
{
    return value == YAW_RATE_UNLIMITED_WIRE ? YAW_RATE_UNLIMITED : value * scale;
}

static inline uint8_t YawLimits_Position(float raw_deg, float *joint_deg)
{
    float joint;
    if (!isfinite(raw_deg) || raw_deg < 0.0f || raw_deg >= 360.0f)
        return 0U;
    joint = remainderf(raw_deg - YAW_LIMIT_ZERO_DEG, 360.0f);
    /* Wrong encoder branch or changed installation: inhibit, do not home. */
    if (joint < YAW_LIMIT_HARD_MIN_DEG - 1.0f ||
        joint > YAW_LIMIT_HARD_MAX_DEG + 1.0f) return 0U;
    *joint_deg = joint;
    return 1U;
}

static inline float YawLimits_Error(float joint, float world_error, float sign)
{
    float lo = fminf(YAW_LIMIT_MIN_DEG - joint, 0.0f);
    float hi = fmaxf(YAW_LIMIT_MAX_DEG - joint, 0.0f);
    /* Include zero outside the soft interval: enabling never commands a jump. */
    return sign * YawLimits_Clamp(sign * world_error, lo, hi);
}

static inline float YawLimits_Speed(float joint, float world_ref_rpm,
                                   float world_fdb_rpm, float joint_fdb_rpm,
                                   float sign, float max_rpm)
{
    float cap = YawLimits_RateCap(max_rpm);
    float brake_rpm = max_rpm == YAW_RATE_UNLIMITED ? YAW_LIMIT_BRAKE_RPM : cap;
    /* Preserve the validated boundary slope, without a flat cruise-speed cap. */
    float lower = -fminf(cap, brake_rpm * fmaxf(
        (joint - YAW_LIMIT_MIN_DEG) / YAW_LIMIT_SLOW_ZONE_DEG, 0.0f));
    float upper = fminf(cap, brake_rpm * fmaxf(
        (YAW_LIMIT_MAX_DEG - joint) / YAW_LIMIT_SLOW_ZONE_DEG, 0.0f));
    /* Limit joint motion, not inertial speed: the base may itself be moving. */
    float base = joint_fdb_rpm - sign * world_fdb_rpm;
    float requested = sign * world_ref_rpm + base;
    float limited = YawLimits_Clamp(requested, lower, upper);
    if (limited == requested) return world_ref_rpm;
    return sign * (limited - base);
}

static inline float YawLimits_Effort(float joint, float effort)
{
    if ((joint >= YAW_LIMIT_MAX_DEG && effort > 0.0f) ||
        (joint <= YAW_LIMIT_MIN_DEG && effort < 0.0f)) return 0.0f;
    return effort;
}

#endif
