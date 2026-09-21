#ifndef MODULE_YAW_MPC_H
#define MODULE_YAW_MPC_H

#include <stdint.h>

#define YAW_MPC_BUILD 0x59490910UL
#define YAW_MPC_PERIOD_MS 20U
#define YAW_MPC_MODEL_REPORT_SHA256 \
    "1E81D3F94F3C81E3CE5A9A04481AABFCFE0270A8753BC4D3CE2C7A7725A2D53D"

typedef enum {
    YAW_MPC_READY = 0,
    YAW_MPC_DISABLED = 1,
    YAW_MPC_STALE_REQUEST = 2,
    YAW_MPC_INVALID_FEEDBACK = 3,
    YAW_MPC_INVALID_STATE = 4,
    YAW_MPC_CALL_GAP = 5
} YawMpc_Reason;

typedef struct {
    uint32_t now_ms;
    uint32_t request_ms;
    uint8_t requested;
    uint8_t feedback_valid;
    float heading_error_deg;
    float small_joint_deg;
    float big_rate_dps;
    float small_inertial_rate_dps;
    float target_heading_rate_dps;
    float previous_big_reference_dps;
    float previous_small_reference_dps;
} YawMpc_Input;

typedef struct {
    uint8_t active;
    uint8_t initialized;
    uint8_t reason;
    uint32_t last_call_ms;
    uint32_t last_update_ms;
    float y1[2];
    float y2[2];
    float u1[2];
    float u2[2];
    float u3[2];
    float output[2];
    float unconstrained[2];
} YawMpc_State;

typedef struct {
    uint8_t active;
    uint8_t updated;
    uint8_t reason;
    float big_reference_dps;
    float small_reference_dps;
} YawMpc_Output;

void YawMpc_Reset(YawMpc_State *state, YawMpc_Reason reason);
YawMpc_Output YawMpc_Update(YawMpc_State *state, const YawMpc_Input *input);
void YawMpc_CommitApplied(YawMpc_State *state,
                          float big_reference_dps,
                          float small_reference_dps);

#endif
