#include "alg_chassis_ramp.h"

#include <math.h>

#define CHASSIS_RAMP_EPSILON (1.0e-6f)

static float ChassisRamp_ClampDt(float dt_s, float maximum_dt_s)
{
    if (dt_s <= 0.0f) return 0.0f;
    if (maximum_dt_s > 0.0f && dt_s > maximum_dt_s) return maximum_dt_s;
    return dt_s;
}

static float ChassisRamp_StepScalar(float current,
                                    float target,
                                    float accel_per_s,
                                    float decel_per_s,
                                    float stop_per_s,
                                    float dt_s)
{
    float delta = target - current;
    float rate;
    float maximum_step;

    if (fabsf(delta) <= CHASSIS_RAMP_EPSILON || dt_s <= 0.0f) return current;

    /* Reversal, release and a smaller same-direction request all use braking. */
    if (fabsf(target) <= CHASSIS_RAMP_EPSILON) {
        rate = stop_per_s;
    } else {
        rate = ((current * delta) < 0.0f) ? decel_per_s : accel_per_s;
    }
    if (rate <= 0.0f) return target;
    maximum_step = rate * dt_s;
    if (delta > maximum_step) return current + maximum_step;
    if (delta < -maximum_step) return current - maximum_step;
    return target;
}

void ChassisRamp_Reset(ChassisRamp_StateTypeDef *state)
{
    if (state == 0) return;
    state->vx = 0.0f;
    state->vy = 0.0f;
    state->wz = 0.0f;
}

void ChassisRamp_Step(ChassisRamp_StateTypeDef *state,
                      float target_vx,
                      float target_vy,
                      float target_wz,
                      float dt_s,
                      const ChassisRamp_ConfigTypeDef *config)
{
    float delta_vx;
    float delta_vy;
    float delta_length;
    float rate;
    float maximum_step;
    float clamped_dt;
    float target_length;

    if (state == 0 || config == 0) return;
    clamped_dt = ChassisRamp_ClampDt(dt_s, config->maximum_dt_s);

    delta_vx = target_vx - state->vx;
    delta_vy = target_vy - state->vy;
    delta_length = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);
    target_length = sqrtf(target_vx * target_vx + target_vy * target_vy);
    if (delta_length > CHASSIS_RAMP_EPSILON && clamped_dt > 0.0f) {
        /* A negative projection removes present velocity, so use deceleration. */
        if (target_length <= CHASSIS_RAMP_EPSILON) {
            rate = config->translation_stop_per_s;
        } else {
            rate = ((state->vx * delta_vx + state->vy * delta_vy) < 0.0f)
                       ? config->translation_decel_per_s
                       : config->translation_accel_per_s;
        }
        if (rate <= 0.0f) {
            state->vx = target_vx;
            state->vy = target_vy;
        } else {
            maximum_step = rate * clamped_dt;
            if (delta_length <= maximum_step) {
                state->vx = target_vx;
                state->vy = target_vy;
            } else {
                state->vx += delta_vx * maximum_step / delta_length;
                state->vy += delta_vy * maximum_step / delta_length;
            }
        }
    }

    state->wz = ChassisRamp_StepScalar(state->wz, target_wz,
                                       config->yaw_accel_per_s,
                                       config->yaw_decel_per_s,
                                       config->yaw_stop_per_s,
                                       clamped_dt);
}
