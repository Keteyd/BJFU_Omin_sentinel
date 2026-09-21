#ifndef MODULE_YAW_COORDINATOR_H
#define MODULE_YAW_COORDINATOR_H

#include "module_yaw_limits.h"

/* Set to 0 to compare the captured HOLD / boundary-relief policy. */
#ifndef YAW_COORD_IMMEDIATE_FOLLOW
#define YAW_COORD_IMMEDIATE_FOLLOW 1U
#endif

#define YAW_COORD_LEFT_ENTER_DEG (YAW_LIMIT_MAX_DEG * 0.70f)
#define YAW_COORD_LEFT_EXIT_DEG (YAW_LIMIT_MAX_DEG * 0.45f)
#define YAW_COORD_RIGHT_ENTER_DEG (YAW_LIMIT_MIN_DEG * 0.70f)
#define YAW_COORD_RIGHT_EXIT_DEG (YAW_LIMIT_MIN_DEG * 0.45f)
#define YAW_COORD_CONFIRM_S 0.12f
#define YAW_COORD_PREDICT_S 0.20f
#define YAW_COORD_PREDICT_MAX_DEG 8.0f
#define YAW_COORD_EMERGENCY_MARGIN_DEG 3.0f
#define YAW_COORD_VELOCITY_TAU_S 0.04f
#define YAW_COORD_FEEDFORWARD_TAU_S 0.01f
#define YAW_COORD_STOP_DPS 0.02f
#define YAW_COORD_POSITION_LEAD_DEG 4.0f
#define YAW_COORD_RELIEF_GAIN 2.0f
#define YAW_COORD_EXIT_PAD_DEG 2.0f

typedef enum {
    YAW_COORD_DISABLED = 0,
    YAW_COORD_HOLD = 1,
    YAW_COORD_LEFT = 2,
    YAW_COORD_RIGHT = 3,
    YAW_COORD_BRAKE = 4
} YawCoordinator_Mode;

typedef struct {
    float reference_deg;
    float velocity_dps;
    float feedforward_dps;
    float filtered_small_rate_dps;
    float predicted_small_deg;
    float confirm_s;
    float relief_direction;
    int8_t candidate;
    uint8_t ready;
    YawCoordinator_Mode mode;
} YawCoordinator_State;

static inline float YawCoordinator_Wrap(float deg)
{
    float result = fmodf(deg, 360.0f);
    return result < 0.0f ? result + 360.0f : result;
}

static inline void YawCoordinator_Reset(YawCoordinator_State *s)
{
    *s = (YawCoordinator_State){0};
}

static inline uint8_t YawCoordinator_Step(YawCoordinator_State *s,
    float small_deg, float small_rate_dps, float big_deg,
    float relief_direction, float dt, uint8_t enabled)
{
#if !YAW_COORD_IMMEDIATE_FOLLOW
    float projected, desired = 0.0f, old_speed, next_ref, error, old_ref, moved;
    float applied_feedforward = 0.0f;
    int8_t request = 0;
    uint8_t urgent_left, urgent_right;
#endif
    if (!enabled || !isfinite(small_deg) || !isfinite(small_rate_dps) ||
        !isfinite(big_deg) || big_deg < 0.0f || big_deg >= 360.0f ||
        !isfinite(dt) || dt <= 0.0f || dt > 0.05f ||
        small_deg < YAW_LIMIT_HARD_MIN_DEG - 1.0f ||
        small_deg > YAW_LIMIT_HARD_MAX_DEG + 1.0f ||
        (relief_direction != 1.0f && relief_direction != -1.0f)) {
        YawCoordinator_Reset(s);
        return 0U;
    }
#if YAW_COORD_IMMEDIATE_FOLLOW
    /* Recenter immediately, including the first enabled update. No velocity FF. */
    YawCoordinator_Reset(s);
    s->reference_deg = YawCoordinator_Wrap(big_deg + relief_direction * small_deg);
    s->relief_direction = relief_direction;
    s->predicted_small_deg = small_deg;
    s->ready = 1U;
    s->mode = small_deg > 0.0f ? YAW_COORD_LEFT :
        (small_deg < 0.0f ? YAW_COORD_RIGHT : YAW_COORD_HOLD);
    return 1U;
#else
    if (!s->ready || s->relief_direction != relief_direction) {
        YawCoordinator_Reset(s);
        s->reference_deg = big_deg;
        s->relief_direction = relief_direction;
        s->predicted_small_deg = small_deg;
        s->ready = 1U;
        s->mode = YAW_COORD_HOLD;
        return 1U;
    }

    s->filtered_small_rate_dps += dt / (0.05f + dt) *
        (small_rate_dps - s->filtered_small_rate_dps);
    projected = small_deg + YawLimits_Clamp(
        s->filtered_small_rate_dps * YAW_COORD_PREDICT_S,
        -YAW_COORD_PREDICT_MAX_DEG, YAW_COORD_PREDICT_MAX_DEG);
    s->predicted_small_deg = projected;
    urgent_left = (uint8_t)(small_deg >= YAW_LIMIT_MAX_DEG -
        YAW_COORD_EMERGENCY_MARGIN_DEG || projected >= YAW_LIMIT_MAX_DEG);
    urgent_right = (uint8_t)(small_deg <= YAW_LIMIT_MIN_DEG +
        YAW_COORD_EMERGENCY_MARGIN_DEG || projected <= YAW_LIMIT_MIN_DEG);

    if (urgent_left) s->mode = YAW_COORD_LEFT;
    else if (urgent_right) s->mode = YAW_COORD_RIGHT;
    else if (s->mode == YAW_COORD_LEFT && small_deg <= YAW_COORD_LEFT_EXIT_DEG)
        s->mode = YAW_COORD_BRAKE;
    else if (s->mode == YAW_COORD_RIGHT && small_deg >= YAW_COORD_RIGHT_EXIT_DEG)
        s->mode = YAW_COORD_BRAKE;

    if (s->mode == YAW_COORD_HOLD || s->mode == YAW_COORD_BRAKE) {
        if (fmaxf(small_deg, projected) >= YAW_COORD_LEFT_ENTER_DEG &&
            projected > YAW_COORD_LEFT_EXIT_DEG) request = 1;
        else if (fminf(small_deg, projected) <= YAW_COORD_RIGHT_ENTER_DEG &&
                 projected < YAW_COORD_RIGHT_EXIT_DEG) request = -1;
        if (request != s->candidate || request == 0) s->confirm_s = 0.0f;
        s->candidate = request;
        if (request != 0) s->confirm_s += dt;
        if (s->confirm_s >= YAW_COORD_CONFIRM_S)
            s->mode = request > 0 ? YAW_COORD_LEFT : YAW_COORD_RIGHT;
    } else {
        s->candidate = 0;
        s->confirm_s = 0.0f;
    }

    if (s->mode == YAW_COORD_LEFT)
        /* Equal relative travel produces equal relief demand on either side. */
        desired = fmaxf(0.0f, (fmaxf(small_deg, projected) -
            YAW_COORD_LEFT_EXIT_DEG) * (-YAW_LIMIT_MIN_DEG / YAW_LIMIT_MAX_DEG)
            + YAW_COORD_EXIT_PAD_DEG) * YAW_COORD_RELIEF_GAIN;
    else if (s->mode == YAW_COORD_RIGHT)
        desired = fminf(0.0f, fminf(small_deg, projected) -
            (YAW_COORD_RIGHT_EXIT_DEG + YAW_COORD_EXIT_PAD_DEG)) * YAW_COORD_RELIEF_GAIN;
    desired *= relief_direction;
    old_speed = s->velocity_dps;
    /* Smooth the planned trajectory, not the PID correction. */
    s->velocity_dps += dt / (YAW_COORD_VELOCITY_TAU_S + dt) * (desired - old_speed);
    if (desired == 0.0f && fabsf(s->velocity_dps) < YAW_COORD_STOP_DPS)
        s->velocity_dps = 0.0f;

    /* Fixed HOLD target; only moving trajectories receive a lead guard. */
    if (old_speed != 0.0f || s->velocity_dps != 0.0f) {
        old_ref = s->reference_deg;
        next_ref = s->reference_deg + 0.5f * (old_speed + s->velocity_dps) * dt;
        error = remainderf(next_ref - big_deg, 360.0f);
        s->reference_deg = YawCoordinator_Wrap(big_deg + YawLimits_Clamp(error,
            -YAW_COORD_POSITION_LEAD_DEG, YAW_COORD_POSITION_LEAD_DEG));
        moved = remainderf(s->reference_deg - old_ref, 360.0f) / dt;
        if (moved * s->velocity_dps > 0.0f)
            applied_feedforward = copysignf(
                fminf(fabsf(moved), fabsf(s->velocity_dps)), s->velocity_dps);
    }
    s->feedforward_dps += dt / (YAW_COORD_FEEDFORWARD_TAU_S + dt) *
        (applied_feedforward - s->feedforward_dps);
    s->feedforward_dps = YawLimits_Clamp(s->feedforward_dps,
        -fabsf(s->velocity_dps), fabsf(s->velocity_dps));
    if (s->feedforward_dps * s->velocity_dps < 0.0f) s->feedforward_dps = 0.0f;
    if (s->mode == YAW_COORD_BRAKE && s->velocity_dps == 0.0f)
        s->mode = YAW_COORD_HOLD;
    return 1U;
#endif
}

#endif
