#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "module_yaw_mpc.h"
#include "../AGVSentinel_gimbal/Src/Modules/module_yaw_mpc.c"

static YawMpc_Input valid_input(uint32_t now)
{
    YawMpc_Input input = {0};
    input.now_ms = now;
    input.request_ms = now;
    input.requested = 1U;
    input.feedback_valid = 1U;
    return input;
}

int main(void)
{
    YawMpc_State state;
    YawMpc_Input input;
    YawMpc_Output output;
    YawMpc_Reset(&state, YAW_MPC_DISABLED);

    input = valid_input(100U);
    input.requested = 0U;
    output = YawMpc_Update(&state, &input);
    assert(!output.active && output.reason == YAW_MPC_DISABLED);

    input = valid_input(120U);
    input.heading_error_deg = 10.0f;
    output = YawMpc_Update(&state, &input);
    assert(output.active && output.updated);
    assert(output.big_reference_dps > 1.0f && output.big_reference_dps < 1.5f);
    assert(output.small_reference_dps > 150.0f);

    input = valid_input(125U);
    input.heading_error_deg = 10.0f;
    output = YawMpc_Update(&state, &input);
    assert(output.active && !output.updated);

    input = valid_input(140U);
    input.heading_error_deg = 10.0f;
    output = YawMpc_Update(&state, &input);
    assert(output.updated && output.big_reference_dps > 1.0f);
    assert(output.small_reference_dps > 60.0f);

    YawMpc_CommitApplied(&state, 400.0f, 500.0f);
    assert(fabsf(state.output[0] - 400.0f) < 0.0001f);
    assert(fabsf(state.output[1] - 500.0f) < 0.0001f);

    input = valid_input(160U);
    input.small_joint_deg = 26.080078125f;
    input.heading_error_deg = 20.0f;
    output = YawMpc_Update(&state, &input);
    assert(output.active);
    assert(output.small_reference_dps - output.big_reference_dps <= 0.0001f);

    input = valid_input(180U);
    input.big_rate_dps = NAN;
    output = YawMpc_Update(&state, &input);
    assert(!output.active && output.reason == YAW_MPC_INVALID_FEEDBACK);
    assert(output.big_reference_dps == 0.0f && output.small_reference_dps == 0.0f);

    input = valid_input(200U);
    input.request_ms = 170U;
    output = YawMpc_Update(&state, &input);
    assert(!output.active && output.reason == YAW_MPC_STALE_REQUEST);

    input = valid_input(220U);
    output = YawMpc_Update(&state, &input);
    assert(output.active);
    input = valid_input(400U);
    output = YawMpc_Update(&state, &input);
    assert(!output.active && output.reason == YAW_MPC_CALL_GAP);

    YawMpc_Reset(&state, YAW_MPC_DISABLED);
    input = valid_input(420U);
    input.target_heading_rate_dps = 100.0f;
    output = YawMpc_Update(&state, &input);
    assert(output.active && output.updated);
    assert(output.big_reference_dps > 2.0f && output.big_reference_dps < 3.0f);
    assert(output.small_reference_dps > 90.0f);

    puts("PASS: unbounded speed-reference MPC, joint envelope and fallback");
    return 0;
}
