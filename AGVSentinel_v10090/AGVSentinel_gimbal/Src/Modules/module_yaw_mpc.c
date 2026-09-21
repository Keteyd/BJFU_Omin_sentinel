#include "module_yaw_mpc.h"

#include <math.h>
#include <string.h>

#define YAW_MPC_STATE_COUNT 14U
#define YAW_MPC_REQUEST_TIMEOUT_MS 20U
#define YAW_MPC_CALL_GAP_MS 100U
#define YAW_MPC_JOINT_MIN_DEG (-53.677734375f)
#define YAW_MPC_JOINT_MAX_DEG (26.080078125f)
#define YAW_MPC_OUTPUT_BASELINE_SMALL_DPS (-0.024253266912407853f)

/* Generated from the accepted S1 report with frequency-selective big-yaw
 * smoothing: joint weight 5, big-reference delta weight 10.0 and big-reference
 * second-difference weight 40.0. u = -K x, 20 ms, 15-step horizon. */
static const float s_gain[2][YAW_MPC_STATE_COUNT] = {
    {-0.116743067f, -0.101148737f, 0.001892264f, 0.002001555f,
     -0.000220786f, 0.000168321f, -1.491589613f, -0.000668640f,
     0.539573675f, -0.000561560f, 0.000743988f, -0.000052321f,
     -0.023017739f, -0.000073887f},
    {-15.966065635f, 0.297914042f, 0.027888658f, 0.690776485f,
     -0.004858016f, 0.100652493f, -0.060462953f, -0.556110636f,
     0.002868135f, -0.202620362f, 0.069883820f, -0.096830422f,
     -1.004999660f, -0.024828475f}
};

static float YawMpc_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static uint8_t YawMpc_FiniteInput(const YawMpc_Input *input)
{
    return (uint8_t)(isfinite(input->heading_error_deg) &&
        isfinite(input->small_joint_deg) && isfinite(input->big_rate_dps) &&
        isfinite(input->small_inertial_rate_dps) &&
        isfinite(input->target_heading_rate_dps) &&
        isfinite(input->previous_big_reference_dps) &&
        isfinite(input->previous_small_reference_dps));
}

static YawMpc_Output YawMpc_Result(const YawMpc_State *state, uint8_t updated)
{
    YawMpc_Output result;
    result.active = state->active;
    result.updated = updated;
    result.reason = state->reason;
    result.big_reference_dps = state->active ? state->output[0] : 0.0f;
    result.small_reference_dps = state->active ? state->output[1] : 0.0f;
    return result;
}

void YawMpc_Reset(YawMpc_State *state, YawMpc_Reason reason)
{
    if (state == 0) return;
    memset(state, 0, sizeof(*state));
    state->reason = (uint8_t)reason;
}

YawMpc_Output YawMpc_Update(YawMpc_State *state, const YawMpc_Input *input)
{
    float x[YAW_MPC_STATE_COUNT];
    float next[2];
    float relative;
    unsigned row, column;

    if (state == 0 || input == 0) {
        YawMpc_Output empty = {0U, 0U, YAW_MPC_INVALID_STATE, 0.0f, 0.0f};
        return empty;
    }
    if (!input->requested) {
        YawMpc_Reset(state, YAW_MPC_DISABLED);
        return YawMpc_Result(state, 0U);
    }
    if (input->now_ms - input->request_ms > YAW_MPC_REQUEST_TIMEOUT_MS) {
        YawMpc_Reset(state, YAW_MPC_STALE_REQUEST);
        return YawMpc_Result(state, 0U);
    }
    if (!input->feedback_valid || !YawMpc_FiniteInput(input)) {
        YawMpc_Reset(state, YAW_MPC_INVALID_FEEDBACK);
        return YawMpc_Result(state, 0U);
    }
    if (state->initialized && input->now_ms - state->last_call_ms > YAW_MPC_CALL_GAP_MS) {
        YawMpc_Reset(state, YAW_MPC_CALL_GAP);
        return YawMpc_Result(state, 0U);
    }
    state->last_call_ms = input->now_ms;

    if (!state->initialized) {
        state->y1[0] = state->y2[0] = input->big_rate_dps;
        state->y1[1] = state->y2[1] =
            input->small_inertial_rate_dps - YAW_MPC_OUTPUT_BASELINE_SMALL_DPS;
        state->u1[0] = state->u2[0] = state->u3[0] =
            input->previous_big_reference_dps;
        state->u1[1] = state->u2[1] = state->u3[1] =
            input->previous_small_reference_dps;
        state->output[0] = state->u1[0];
        state->output[1] = state->u1[1];
        state->initialized = 1U;
        state->active = 1U;
        state->reason = YAW_MPC_READY;
        state->last_update_ms = input->now_ms - YAW_MPC_PERIOD_MS;
    }
    if (input->now_ms - state->last_update_ms < YAW_MPC_PERIOD_MS)
        return YawMpc_Result(state, 0U);

    x[0] = remainderf(input->heading_error_deg, 360.0f);
    x[1] = input->small_joint_deg;
    x[2] = input->big_rate_dps;
    x[3] = input->small_inertial_rate_dps - YAW_MPC_OUTPUT_BASELINE_SMALL_DPS;
    x[4] = state->y1[0]; x[5] = state->y1[1];
    x[6] = state->u1[0]; x[7] = state->u1[1];
    x[8] = state->u2[0]; x[9] = state->u2[1];
    x[10] = state->u3[0]; x[11] = state->u3[1];
    x[12] = input->target_heading_rate_dps;
    x[13] = 1.0f;
    for (row = 0U; row < 2U; ++row) {
        float value = 0.0f;
        for (column = 0U; column < YAW_MPC_STATE_COUNT; ++column)
            value -= s_gain[row][column] * x[column];
        if (!isfinite(value)) {
            YawMpc_Reset(state, YAW_MPC_INVALID_STATE);
            return YawMpc_Result(state, 0U);
        }
        state->unconstrained[row] = value;
    }

    /* Operator-selected cable-free deployment: apply the frozen first move
     * without an MPC-domain absolute-speed or slew-rate clamp.  The existing
     * speed-PID effort limits remain authoritative, and the relative-speed
     * projection below still prevents commanding the small joint through its
     * measured mechanical envelope. */
    next[0] = state->unconstrained[0];
    next[1] = state->unconstrained[1];
    relative = YawMpc_Clamp(next[1] - next[0],
        (YAW_MPC_JOINT_MIN_DEG - input->small_joint_deg) / 0.020f,
        (YAW_MPC_JOINT_MAX_DEG - input->small_joint_deg) / 0.020f);
    next[1] = next[0] + relative;

    state->y2[0] = state->y1[0]; state->y2[1] = state->y1[1];
    state->y1[0] = x[2]; state->y1[1] = x[3];
    state->u3[0] = state->u2[0]; state->u3[1] = state->u2[1];
    state->u2[0] = state->u1[0]; state->u2[1] = state->u1[1];
    state->u1[0] = next[0]; state->u1[1] = next[1];
    state->output[0] = next[0]; state->output[1] = next[1];
    state->last_update_ms = input->now_ms;
    state->active = 1U;
    state->reason = YAW_MPC_READY;
    return YawMpc_Result(state, 1U);
}

void YawMpc_CommitApplied(YawMpc_State *state,
                          float big_reference_dps,
                          float small_reference_dps)
{
    if (state == 0 || !state->active || !isfinite(big_reference_dps) ||
        !isfinite(small_reference_dps)) return;
    state->u1[0] = state->output[0] = big_reference_dps;
    state->u1[1] = state->output[1] = small_reference_dps;
}
