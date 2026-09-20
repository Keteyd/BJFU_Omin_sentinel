#include <assert.h>
#include <stdio.h>
#include "ident_fixture.h"
#define YAW_IDENT_MOTION_ENABLED 1U
#include "../AGVSentinel_gimbal/Src/Application/app_yaw_identification.c"

/* Run the production supervisor with fake clocks, remote, sensors and transport. */
static uint32_t clock_ms, irq_mask;
static uint8_t trace_dma_paced;
static uint32_t trace_dma_ready_ms;
static uint8_t ready = 1U, tune_online, tx_ready = 1U;
static uint8_t mock_dm_lifecycle;
static float pitch_config_kp, pitch_config_kd;
static uint8_t remote_wire[3][12];
static unsigned remote_frames;
static YawIdent_Record streamed[YAW_IDENT_CD_SAMPLES];
static unsigned stream_count;
static FILE *stream_wire_fixture;
static const char *stream_fixture_path;
static const char *dual_stream_fixture_path;
static const char *cd_stream_fixture_path;
static const char *e_stream_fixture_path;
static const char *speed_stream_fixture_path;
static const char *speed_s3_stream_fixture_path;
static CanTrace_State trace_state;
static uint32_t trace_started_ms;
static uint8_t trace_fixture_fault;
void Can_TraceBegin(void) { CanTrace_Begin(&trace_state); trace_started_ms = clock_ms; }
void Can_TraceEnd(void) { trace_state.active = 0U; }
uint32_t Can_TraceSnapshot(uint32_t *now, CanTrace_AxisRecord out[2]) {
    unsigned i; uint8_t feedback[8] = {0};
    *now = (clock_ms - trace_started_ms)*1000U;
    for (i = 0; i < 2U; ++i) {
        uint32_t seq = CanTrace_Attempt(&trace_state, i, 0, *now);
        CanTrace_Queue(&trace_state, i, seq, i*3U, 1U, 0U, 0U);
        CanTrace_Complete(&trace_state, i*3U, 0U, *now);
        CanTrace_Feedback(&trace_state, i, feedback, *now);
    }
    {
        uint32_t interval = CanTrace_Snapshot(&trace_state, *now, out);
        if (trace_fixture_fault == 1U) out[0].flags &= ~CT_FEEDBACK;
        if (trace_fixture_fault == 2U) out[0].maximum = 1;
        if (trace_fixture_fault == 3U) out[0].flags |= CT_BUS_ERROR;
        if (trace_fixture_fault == 4U && out[0].known_us) --out[0].known_us;
        return interval;
    }
}
uint8_t PC_Comm_SendTrace(uint32_t id, uint16_t sequence, const void *record) {
    uint8_t bytes[CAN_TRACE_FRAME_BYTES];
    if (!tx_ready) return 0U;
    if (trace_dma_paced && (int32_t)(clock_ms - trace_dma_ready_ms) < 0) return 0U;
    if (trace_dma_paced) trace_dma_ready_ms = clock_ms + 4U;
    assert(sequence == stream_count + 1U && sequence <= YAW_IDENT_CD_SAMPLES);
    streamed[stream_count++] = ((const YawIdent_TraceRecord *)record)->sample;
    CanTrace_Frame(bytes, id, sequence, record);
    if (stream_wire_fixture) assert(fwrite(bytes, 1U, sizeof(bytes), stream_wire_fixture) == sizeof(bytes));
    return 1U;
}

static void fixture_packet(uint8_t cmd, const uint8_t payload[12]) {
    uint8_t bytes[16] = {0xffU, cmd}, crc = 0U;
    unsigned i, bit;
    if (!stream_wire_fixture) return;
    memcpy(bytes + 2U, payload, 12U);
    for (i = 0U; i < 14U; ++i) {
        crc ^= bytes[i];
        for (bit = 0U; bit < 8U; ++bit)
            crc = (uint8_t)((crc << 1U) ^ ((crc & 0x80U) ? 0x31U : 0U));
    }
    bytes[14] = crc; bytes[15] = 0x0dU;
    assert(fwrite(bytes, 1U, sizeof(bytes), stream_wire_fixture) == sizeof(bytes));
}
static INS_Observation observation;
static Remote_RemoteDataTypeDef remote;
static GimbalYaw_GimbalYawTypeDef yaw;
static GimbalPitch_GimbalPitchTypeDef pitch;
MockDm motor[1];
Motor_MotorTypeDef Motor_Big_YawMotor, Motor_Small_YawMotor;
uint8_t GimbalYaw_DiagSmallEnable, GimbalYaw_DiagBigActive, GimbalYaw_DiagBigSaturated, GimbalPitch_ImuEnable;
uint8_t GimbalYaw_DiagBigStopReason;
uint8_t GimbalPitch_DiagMotorOnline;
float GimbalYaw_TuneBigAngKp;
float GimbalYaw_TuneBigSpdKp;
float GimbalYaw_TuneBigEffortLimit;
float GimbalYaw_TuneBigSpeedFilterTauS;
float GimbalYaw_TuneBigAngKi;
float GimbalYaw_TuneBigAngKd;
float GimbalYaw_TuneBigSpdKi;
float GimbalYaw_TuneBigSpdKd;
float GimbalYaw_TuneSmallAngKp;
float GimbalYaw_TuneSmallSpdKp;
float GimbalYaw_DiagEffortLimit;
float GimbalYaw_TuneSmallSpeedFilterAlpha;
float GimbalYaw_TuneSmallAngKi;
float GimbalYaw_TuneSmallAngKd;
float GimbalYaw_TuneSmallSpdKi;
float GimbalYaw_TuneSmallSpdKd;
float GimbalYaw_TuneSmallSpeedLimitRpm;
float GimbalYaw_TuneSmallManualMaxStepDeg;
float GimbalYaw_DiagSmallDirection;
float GimbalYaw_DiagBigDirection;
float GimbalYaw_DiagBigSpeedRefRpm;
float GimbalYaw_DiagSmallSpeedRefRpm;
float GimbalPitch_TuneImuKp;
float GimbalPitch_TuneImuKd;
float GimbalPitch_TuneGravityEffort;
float GimbalPitch_TuneRateLimitRadS;
float GimbalPitch_TuneAngleDeadbandRad;
float GimbalPitch_TuneRateDeadbandRadS;
float GimbalYaw_TuneSmallStepDeg;
float GimbalPitch_TuneStepRad;
uint32_t HAL_GetTick(void) { return clock_ms; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }
void __DMB(void) {}
void INS_ReadObservation(INS_Observation *o) { *o = observation; }
uint8_t INS_IsReady(void) { return ready; }
Remote_RemoteDataTypeDef *Remote_GetRemoteDataPtr(void) { return &remote; }
GimbalYaw_GimbalYawTypeDef *GimbalYaw_GetGimbalYawPtr(void) { return &yaw; }
GimbalPitch_GimbalPitchTypeDef *GimbalPitch_GetGimbalPitchPtr(void) { return &pitch; }
void GimbalYaw_SetGimbalYawOutputState(uint8_t state) { yaw.output_state = state; }
void GimbalPitch_SetGimbalPitchOutputState(uint8_t state) { pitch.output_state = state; }
void GimbalYaw_SetSmallYawRateDps(float rate) { assert(rate == 0.0f); }
float GimbalPitch_GetPositionFeedback(void) { return 0.0f; }
void GimbalYaw_Output(void) {
    YawIdentApp_ValidateOutput();
    if (!yaw.output_state) Motor_Big_YawMotor.output = Motor_Small_YawMotor.output = 0.0f;
}
void GimbalPitch_Output(void) {
    if (mock_dm_lifecycle) {
        motor[Motor1].ctrl.kp_set = pitch.output_state ? pitch_config_kp : 0.0f;
        motor[Motor1].ctrl.kd_set = pitch.output_state ? pitch_config_kd : 0.0f;
    }
}
void RobotActuators_GetPitchMitGains(float *kp, float *kd) {
    if (kp) *kp = pitch_config_kp;
    if (kd) *kd = pitch_config_kd;
}
uint8_t PC_Comm_IsRemoteSafe(void) {
    return remote.state == Remote_STATE_CONNECTED && remote.remote.s[1] == Remote_SWITCH_UP &&
           clock_ms - remote.last_update_time <= 50U;
}
uint8_t PC_Comm_IsGimbalTuneSessionOnline(void) { return tune_online; }
uint8_t PC_Comm_SendPacket(uint8_t cmd, const void *data, uint8_t size) {
    assert(size == 12U);
    if (trace_dma_paced && (int32_t)(clock_ms - trace_dma_ready_ms) < 0) return 0U;
    if (trace_dma_paced && tx_ready) trace_dma_ready_ms = clock_ms + 1U;
    if (tx_ready) fixture_packet(cmd, data);
    if (tx_ready && cmd == YAW_IDENT_CMD_REMOTE) {
        assert(remote_frames < 3U);
        memcpy(remote_wire[remote_frames++], data, 12U);
    }
    return tx_ready;
}
uint8_t PC_Comm_SendBatch(uint8_t cmd, const uint8_t *data, uint8_t count) {
    unsigned i;
    uint16_t sequence;
    assert(cmd == YAW_IDENT_CMD_STREAM && count == 6U);
    if (!tx_ready) return 0U;
    memcpy(&sequence, data, 2U);
    assert(sequence == stream_count + 1U && sequence <= YAW_IDENT_CD_SAMPLES);
    for (i = 0U; i < 6U; ++i) {
        assert(data[i * 12U + 2U] == i && data[i * 12U + 3U] == 2U);
        memcpy((uint8_t *)&streamed[stream_count] + i * 8U, data + i * 12U + 4U, 8U);
        fixture_packet(cmd, data + i * 12U);
    }
    ++stream_count;
    return 1U;
}
static void fresh(uint32_t now) {
    clock_ms = now;
    remote.last_update_time = observation.tick_ms = now;
    Motor_Big_YawMotor.last_update_time = Motor_Small_YawMotor.last_update_time = now;
}
static void setup(void) {
    memset(&YawIdent_StorageData, 0, sizeof(YawIdent_StorageData));
    memset(&s_state, 0, sizeof(s_state));
    memset(&s_ref, 0, sizeof(s_ref));
    memset(&s_meta, 0, sizeof(s_meta));
    memset(&observation, 0, sizeof(observation));
    memset(&remote, 0, sizeof(remote));
    memset(&yaw, 0, sizeof(yaw)); memset(&pitch, 0, sizeof(pitch));
    s_head = s_tail = s_overflow = s_owned = s_cancel = s_recording = s_download = 0U;
    s_seen = s_info_pending = s_ack_pending = s_meta_part = s_part = 0U;
    s_send_index = 0U; s_seen_time = s_last_sample = s_status_time = 0U;
    s_remote_pending = s_remote_part = 0U; s_remote_sequence = 0U; remote_frames = 0U;
    s_count = 0U; tune_online = 0U; ready = 1U; tx_ready = 1U;
    trace_fixture_fault = 0U;
    s_ack_index = 0U; s_profile_part = s_stream_timing_bad = 0U;
    s_stream_progress_ms = 0U; stream_count = 0U;
    mock_dm_lifecycle = 0U; pitch_config_kp = 50.0f; pitch_config_kd = 3.2f;
    memset(motor, 0, sizeof(motor));
    Motor_Big_YawMotor = (Motor_MotorTypeDef){0};
    Motor_Small_YawMotor = (Motor_MotorTypeDef){0};
    Motor_Big_YawMotor.encoder.limited_angle = 359.0f;
    Motor_Small_YawMotor.encoder.limited_angle = YAW_LIMIT_ZERO_DEG;
    Motor_Small_YawMotor.encoder.angle = 6816U;
    Motor_Big_YawMotor.is_online = Motor_Small_YawMotor.is_online = 1U;
    remote.state = Remote_STATE_CONNECTED; remote.remote.s[1] = Remote_SWITCH_UP;
    yaw.control_state = pitch.control_state = 1U;
    GimbalYaw_DiagSmallEnable = GimbalPitch_ImuEnable = GimbalYaw_DiagBigActive = 1U;
    GimbalPitch_DiagMotorOnline = 1U;
    GimbalYaw_DiagEffortLimit = 6.0f; GimbalYaw_TuneBigEffortLimit = 30.0f;
    GimbalYaw_DiagSmallDirection = GimbalYaw_DiagBigDirection = 1.0f;
    GimbalYaw_TuneBigAngKp = 50.0f;
    fresh(100U);
}
static void command(uint8_t op, uint32_t id, uint8_t axis) {
    YawIdent_Packet p = {id, op == YAW_IDENT_ARM ? 100U : 0U, op, axis, YAW_IDENT_MAGIC};
    YawIdentApp_Receive((const uint8_t *)&p);
}
static void cycle(uint32_t now) {
    fresh(now);
    YawIdentApp_Pre();
    if (YawIdentApp_UsesSpeedReference() && YawIdentApp_AllowYaw()) {
        GimbalYaw_DiagBigSpeedRefRpm = YawIdentApp_BigSpeedReferenceRpm();
        GimbalYaw_DiagSmallSpeedRefRpm = YawIdentApp_SmallSpeedReferenceRpm();
    } else {
        GimbalYaw_DiagBigSpeedRefRpm = 0.0f;
        GimbalYaw_DiagSmallSpeedRefRpm = 0.0f;
    }
    GimbalYaw_Output();
    GimbalPitch_Output();
    YawIdentApp_Post();
}
static void arm(uint8_t axis) {
    command(YAW_IDENT_ARM, 1U, axis);
    cycle(clock_ms);
    assert(s_owned && s_state.phase == YAW_IDENT_ARMED);
    assert(!yaw.output_state && !pitch.output_state && !YawIdentApp_AllowYaw());
}
static void start(void) {
    remote.remote.s[1] = Remote_SWITCH_DOWN;
    cycle(clock_ms + 4U);
    assert(YawIdentApp_AllowYaw() && yaw.output_state && pitch.output_state && s_count == 1U);
}
static void full_trial(uint8_t axis, uint32_t base, FILE *fixture) {
    unsigned i;
    setup(); fresh(base); arm(axis); start();
    for (i = 4U; i <= 4000U; i += 4U) {
        fresh(base + 4U + i);
        if (i % 100U == 0U) command(YAW_IDENT_KEEPALIVE, 1U, 0U);
        cycle(clock_ms);
    }
    assert(s_state.phase == YAW_IDENT_DONE && s_count == 1001U);
    assert(s_owned && !yaw.output_state && !pitch.output_state && !YawIdentApp_AllowYaw());
    for (i = 0U; i < 30U; ++i) cycle(clock_ms + 4U);
    assert(s_count == 1001U);
    command(YAW_IDENT_RELEASE, 1U, 0U); cycle(clock_ms);
    assert(s_owned);
    remote.remote.s[1] = Remote_SWITCH_UP; cycle(clock_ms + 4U);
    command(YAW_IDENT_DOWNLOAD, 1U, 0U); cycle(clock_ms);
    assert(s_download && s_meta.count == 1001U);
    if (fixture) {
        assert(fwrite(&s_meta, sizeof(s_meta), 1U, fixture) == 1U);
        assert(fwrite(YawIdent_StorageData.records, sizeof(YawIdent_Record), s_count, fixture) == s_count);
    }
    command(YAW_IDENT_CANCEL, 1U, 0U); cycle(clock_ms);
    assert(s_state.phase == YAW_IDENT_DONE);
    command(YAW_IDENT_RELEASE, 1U, 0U); cycle(clock_ms);
    assert(!s_owned && s_state.last_id == 1U);
    command(YAW_IDENT_ARM, 1U, axis); cycle(clock_ms);
    assert(!s_owned);
}
static void failures(void) {
    unsigned mode, i;
    for (mode = 0U; mode < 12U; ++mode) {
        uint16_t frozen;
        setup(); arm(1U); start();
        switch (mode) {
        case 0: remote.remote.s[1] = Remote_SWITCH_UP; break;
        case 1: remote.remote.ch[0] = 20; break;
        case 2: observation.gyro[2] = NAN; break;
        case 3: GimbalYaw_TuneBigAngKp = 51.0f; break;
        case 4: command(YAW_IDENT_CANCEL, 1U, 0U); assert(!YawIdentApp_AllowYaw()); break;
        case 5: for (i = 0; i < 8U; ++i) command(YAW_IDENT_PROBE, 0U, 0U); break;
        case 6: Motor_Big_YawMotor.encoder.limited_angle = 20.0f; break;
        case 7: GimbalYaw_DiagBigActive = 0U; break;
        case 8: Motor_Big_YawMotor.output = NAN; break;
        case 9: observation.roll_deg = 6.0f; break;
        case 10:
            GimbalPitch_DiagMotorOnline = 0U;
            for (i = 0U; i < 130U; ++i) {
                fresh(clock_ms + 4U);
                if (i % 25U == 0U) command(YAW_IDENT_KEEPALIVE, 1U, 0U);
                cycle(clock_ms);
            }
            break;
        default:
            for (i = 0; i < 76U; ++i) cycle(clock_ms + 4U);
            break;
        }
        cycle(clock_ms + 4U);
        assert(s_owned && s_state.phase == YAW_IDENT_ABORTED);
        if (mode == 7U) assert(s_state.reason == YAW_IDENT_YAW_CONTROL_INACTIVE);
        if (mode == 8U) assert(s_state.reason == YAW_IDENT_OUTPUT_NONFINITE);
        if (mode == 10U) assert(s_state.reason == YAW_IDENT_PITCH_OFFLINE);
        assert(!yaw.output_state && !pitch.output_state);
        assert(Motor_Big_YawMotor.output == 0.0f && Motor_Small_YawMotor.output == 0.0f);
        frozen = s_count;
        for (i = 0; i < 20U; ++i) cycle(clock_ms + 4U);
        assert(s_count == frozen);
    }
    setup(); tune_online = 1U;
    command(YAW_IDENT_ARM, 1U, 1U); cycle(clock_ms); assert(!s_owned);
    setup(); remote.remote.s[1] = Remote_SWITCH_DOWN;
    command(YAW_IDENT_ARM, 1U, 1U); cycle(clock_ms);
    assert(s_owned && s_state.phase == YAW_IDENT_ARMED && !yaw.output_state);
    remote.remote.s[1] = Remote_SWITCH_UP; cycle(clock_ms + 1U);
    remote.remote.s[1] = Remote_SWITCH_DOWN; cycle(clock_ms + 1U);
    assert(s_state.phase == YAW_IDENT_BASELINE && yaw.output_state);
    setup(); arm(2U);
    command(YAW_IDENT_CANCEL, 999U, 0U); cycle(clock_ms); assert(s_state.phase == YAW_IDENT_ARMED);
    cycle(clock_ms + 11U); assert(s_state.reason == YAW_IDENT_TIMING);
    setup(); arm(1U); start();
    cycle(clock_ms + 5U);
    assert(!(YawIdent_StorageData.records[1].flags & (1U << 11)) && !s_stream_timing_bad);
}
static void remote_diagnostics(const char *fixture_path) {
    unsigned i;
    YawIdent_RemoteRecord decoded;
    setup();
    for (i = 0U; i < 4U; ++i) {
        remote.remote.ch[i] = -10; assert(Ident_Neutral(&remote));
        remote.remote.ch[i] = 10; assert(Ident_Neutral(&remote));
        remote.remote.ch[i] = 11;
        assert(YawIdent_NeutralMask(remote.remote.ch, 0, 0, 0, 0) == (1U << i));
        remote.remote.ch[i] = -11; assert(!Ident_Neutral(&remote));
        remote.remote.ch[i] = 0;
    }
    remote.remote.ch[4] = -660; assert(Ident_Neutral(&remote));
    remote.remote.ch[4] = 0; assert(Ident_Neutral(&remote));
    remote.remote.ch[4] = 100; assert(Ident_Neutral(&remote));
    remote.remote.ch[4] = 101; assert(!Ident_Neutral(&remote));
    remote.remote.ch[4] = 660; assert(!Ident_Neutral(&remote));
    remote.remote.ch[4] = 0;
    assert(YawIdent_NeutralMask(remote.remote.ch, 1, 0, 0, 0) == (1U << 5));
    assert(YawIdent_NeutralMask(remote.remote.ch, 0, -1, 0, 0) == (1U << 6));
    assert(YawIdent_NeutralMask(remote.remote.ch, 0, 0, 1, 0) == (1U << 7));
    assert(YawIdent_NeutralMask(remote.remote.ch, 0, 0, 0, 1) == (1U << 8));
    remote.remote.ch[0] = -11; remote.remote.ch[1] = -10;
    remote.remote.ch[3] = 10; remote.remote.ch[4] = 101;
    remote.mouse.x = 2; remote.mouse.y = -3; remote.mouse.l = 1;
    command(YAW_IDENT_PROBE, 0U, 0U); cycle(clock_ms);
    assert(!s_owned && !yaw.output_state && !pitch.output_state);
    assert(s_remote_record.failed_mask == 241U);
    memset(remote.remote.ch, 0, sizeof(remote.remote.ch));
    remote.mouse.x = remote.mouse.y = 0; remote.mouse.l = 0;
    tx_ready = 0U;
    command(YAW_IDENT_PROBE, 0U, 0U); cycle(clock_ms + 1U);
    assert(s_remote_record.failed_mask == 241U && remote_frames == 0U);
    tx_ready = 1U;
    for (i = 0U; i < 8U; ++i) cycle(clock_ms + 1U);
    assert(remote_frames == 3U && !s_remote_pending);
    for (i = 0U; i < 3U; ++i) {
        assert(remote_wire[i][0] == 1U && remote_wire[i][1] == 0U);
        assert(remote_wire[i][2] == i && remote_wire[i][3] == 2U);
        memcpy((uint8_t *)&decoded + i * 8U, remote_wire[i] + 4U, 8U);
    }
    assert(decoded.tick_ms == 100U && decoded.failed_mask == 241U);
    assert(decoded.channels[0] == -11 && decoded.channels[2] == 0 && decoded.channels[4] == 101);
    assert(decoded.mouse_x == 2 && decoded.mouse_y == -3 && decoded.mouse_left == 1U);
    if (fixture_path) {
        FILE *f = fopen(fixture_path, "wb"); assert(f);
        assert(fwrite(&decoded, sizeof(decoded), 1U, f) == 1U); fclose(f);
    }
    setup(); arm(1U);
    command(YAW_IDENT_PROBE, 0U, 0U); cycle(clock_ms);
    assert(s_owned && !s_remote_pending && remote_frames == 0U);
}

static void wheel_safety(void) {
    unsigned i;
    setup(); remote.remote.ch[4] = -660; arm(1U); start();
    remote.remote.ch[4] = 100; cycle(clock_ms + 4U);
    assert(YawIdentApp_AllowYaw());
    remote.remote.ch[4] = 101; cycle(clock_ms + 4U);
    assert(s_owned && s_state.phase == YAW_IDENT_ABORTED);
    assert(!yaw.output_state && !pitch.output_state);
    assert(Motor_Big_YawMotor.output == 0.0f && Motor_Small_YawMotor.output == 0.0f);
    setup(); remote.remote.ch[4] = 101;
    command(YAW_IDENT_ARM, 1U, 1U); cycle(clock_ms);
    assert(s_owned && s_state.phase == YAW_IDENT_ABORTED);
    for (i = 0U; i < 4U; ++i) {
        setup(); remote.remote.ch[4] = -660; remote.remote.ch[i] = 11;
        command(YAW_IDENT_ARM, 1U, 1U); cycle(clock_ms);
        assert(s_owned && s_state.phase == YAW_IDENT_ABORTED);
    }
    setup(); remote.remote.ch[4] = -660; remote.mouse.l = 1;
    command(YAW_IDENT_ARM, 1U, 1U); cycle(clock_ms);
    assert(s_owned && s_state.phase == YAW_IDENT_ABORTED);
}

static void switch_transition(void) {
    unsigned i;
    setup(); arm(1U);
    remote.remote.s[1] = Remote_SWITCH_MIDDLE;
    for (i = 0U; i < 25U; ++i) {
        cycle(clock_ms + 4U);
        assert(s_state.phase == YAW_IDENT_ARMED && s_count == 0U);
        assert(!YawIdentApp_AllowYaw() && !yaw.output_state && !pitch.output_state);
    }
    start();
    remote.remote.s[1] = Remote_SWITCH_MIDDLE; cycle(clock_ms + 4U);
    assert(s_state.phase == YAW_IDENT_ABORTED && s_state.reason == YAW_IDENT_OPERATOR_STOP);
    assert(!yaw.output_state && !pitch.output_state);
    for (i = 0U; i < 6U; ++i) {
        setup(); arm(1U); remote.remote.s[1] = Remote_SWITCH_MIDDLE;
        switch (i) {
        case 0: remote.state = 0U; break;
        case 1: remote.remote.s[1] = 0U; break;
        case 2: remote.remote.ch[2] = 11; break;
        case 3: remote.remote.ch[4] = 101; break;
        case 4: Motor_Big_YawMotor.output = 1.0f; break;
        default: command(YAW_IDENT_CANCEL, 1U, 0U); break;
        }
        cycle(clock_ms + 4U);
        assert(s_state.phase == YAW_IDENT_ABORTED);
        assert(!yaw.output_state && !pitch.output_state);
    }
    setup(); arm(1U); remote.remote.s[1] = Remote_SWITCH_MIDDLE;
    for (i = 0U; i < 3800U && s_state.phase == YAW_IDENT_ARMED; ++i) {
        if (i % 25U == 0U) command(YAW_IDENT_KEEPALIVE, 1U, 0U);
        cycle(clock_ms + 4U);
    }
    assert(s_state.reason == YAW_IDENT_ARM_EXPIRED && s_count == 0U);
}

static void pitch_configuration(void) {
    unsigned i;
    setup(); mock_dm_lifecycle = 1U; arm(1U); start();
    cycle(clock_ms + 4U);
    assert(s_state.phase == YAW_IDENT_BASELINE && YawIdentApp_AllowYaw());
    assert(s_meta.values[10U + 27U] == 50.0f && s_meta.values[10U + 28U] == 3.2f);
    for (i = 0U; i < 4000U; i += 4U) {
        if (i % 100U == 0U) command(YAW_IDENT_KEEPALIVE, 1U, 0U);
        cycle(clock_ms + 4U);
    }
    assert(s_state.phase == YAW_IDENT_DONE && s_count == 1001U);
    assert(motor[Motor1].ctrl.kp_set == 0.0f && motor[Motor1].ctrl.kd_set == 0.0f);
    for (i = 0U; i < 4U; ++i) {
        setup(); mock_dm_lifecycle = 1U; arm(1U);
        if (i >= 2U) start();
        if (i & 1U) pitch_config_kd += 0.1f;
        else pitch_config_kp += 1.0f;
        cycle(clock_ms + 4U);
        assert(s_state.phase == YAW_IDENT_ABORTED && s_state.reason == YAW_IDENT_CONFIG_CHANGED);
        assert(!yaw.output_state && !pitch.output_state);
    }
}

static void rate_output_guard(void) {
    int sign;
    for (sign = -1; sign <= 1; sign += 2) {
        setup(); arm(1U); start();
        Motor_Big_YawMotor.encoder.speed = (float)sign * 60.0f;
        assert(YawIdentApp_AllowYaw());
        cycle(clock_ms + 4U);
        assert(yaw.output_state && s_state.phase == YAW_IDENT_BASELINE);
        Motor_Big_YawMotor.encoder.speed = (float)sign * 61.0f;
        assert(!YawIdentApp_AllowYaw());
        YawIdentApp_ValidateOutput();
        assert(!yaw.output_state && !pitch.output_state);
        setup(); arm(2U); start();
        Motor_Small_YawMotor.encoder.speed = (float)sign * 30.0f;
        assert(YawIdentApp_AllowYaw());
        Motor_Small_YawMotor.encoder.speed = (float)sign * 31.0f;
        assert(!YawIdentApp_AllowYaw());
        YawIdentApp_ValidateOutput();
        assert(!yaw.output_state && !pitch.output_state);
    }
}

static void stream_request(uint8_t op, uint32_t id, uint8_t axis, uint16_t value) {
    YawIdent_Packet p = {id, value, op, axis, YAW_IDENT_MAGIC};
    YawIdentApp_Receive((const uint8_t *)&p);
}

static void stream_complete(uint8_t op, uint8_t axis, uint16_t amplitude) {
    uint32_t id = s_state.last_id + 1U, start_tick;
    unsigned i;
    uint8_t bench = (uint8_t)(op == YAW_IDENT_BENCH);
    uint8_t dual = (uint8_t)(op == YAW_IDENT_ARM_DUAL_A || op == YAW_IDENT_ARM_DUAL_B);
    uint8_t cd = (uint8_t)(op == YAW_IDENT_ARM_CD_C || op == YAW_IDENT_ARM_CD_D ||
        op == YAW_IDENT_ARM_CD_E);
    uint8_t speed_profile = (uint8_t)(op == YAW_IDENT_ARM_SPEED_S1 ||
        op == YAW_IDENT_ARM_SPEED_S2 || op == YAW_IDENT_ARM_SPEED_S3);
    uint32_t duration = speed_profile ? YAW_IDENT_SPEED_DURATION_MS :
        (cd ? YAW_IDENT_CD_DURATION_MS : YAW_IDENT_SLOW_DURATION_MS);
    uint16_t samples = speed_profile ? YAW_IDENT_SPEED_SAMPLES :
        (cd ? YAW_IDENT_CD_SAMPLES : YAW_IDENT_SLOW_SAMPLES);
    uint32_t pre = speed_profile ? YAW_IDENT_SPEED_PRE_MS :
        (cd ? YAW_IDENT_CD_PRE_MS : YAW_IDENT_SLOW_PRE_MS);
    uint32_t drive = speed_profile ? YAW_IDENT_SPEED_DRIVE_MS :
        (cd ? YAW_IDENT_CD_DRIVE_MS : YAW_IDENT_SLOW_DRIVE_MS);
    stream_count = 0U;
    stream_request(op, id, axis, amplitude); cycle(clock_ms);
    assert(s_owned);
    if (!bench) {
        assert(s_state.phase == YAW_IDENT_ARMED);
        remote.remote.s[1] = Remote_SWITCH_DOWN; cycle(clock_ms + 1U);
    }
    assert(s_state.phase == YAW_IDENT_BASELINE && s_count == 1U);
    start_tick = clock_ms;
    for (i = 1U; i <= duration + (trace_dma_paced ? 2000U : 200U); ++i) {
        if (i % 100U == 0U) command(YAW_IDENT_KEEPALIVE, id, 0U);
        if (stream_count > s_ack_index && (!trace_dma_paced || i % 20U == 0U))
            stream_request(YAW_IDENT_STREAM_ACK, id, 0U, (uint16_t)stream_count);
        cycle(start_tick + i);
        assert(s_state.phase != YAW_IDENT_ABORTED);
        if (bench || i >= duration) {
            assert(!yaw.output_state && !pitch.output_state && !YawIdentApp_AllowYaw());
            assert(Motor_Big_YawMotor.output == 0.0f && Motor_Small_YawMotor.output == 0.0f);
        }
    }
    assert(s_state.phase == YAW_IDENT_DONE && s_state.reason == YAW_IDENT_OK);
    assert(s_count == samples && s_ack_index == samples && stream_count == samples);
    assert(s_meta.count == samples && s_meta.version == YAW_IDENT_STREAM_VERSION && s_meta.phase == YAW_IDENT_DONE);
    assert(s_profile_record.profile == (bench ? 2U : dual ? 3U : cd ? 4U : speed_profile ? 5U : 1U));
    assert(s_profile_record.samples == samples && s_profile_record.duration_ms == duration);
    for (i = 0U; i < samples; ++i) {
        assert(streamed[i].tick_ms == start_tick + i * 4U);
        assert(!(streamed[i].flags & (1U << 11)));
        if (i < pre / 4U) assert(streamed[i].phase == YAW_IDENT_BASELINE);
        else if (i < (pre + drive) / 4U) assert(streamed[i].phase == YAW_IDENT_EXCITE);
        else if (i < duration / 4U) assert(streamed[i].phase == YAW_IDENT_SETTLE);
        else assert(streamed[i].phase == YAW_IDENT_DONE);
    }
    if (dual || cd || speed_profile) {
        uint8_t phase_set = (op == YAW_IDENT_ARM_CD_E || op == YAW_IDENT_ARM_SPEED_S3) ? 2U :
            (uint8_t)(op == YAW_IDENT_ARM_DUAL_B || op == YAW_IDENT_ARM_CD_D ||
                op == YAW_IDENT_ARM_SPEED_S2);
        float profile_scale = cd ? (phase_set == 2U ? 5.0f : phase_set == 1U ? 4.0f : 1.0f) : 1.0f;
        assert(s_profile_record.big_peak == (speed_profile ? 30.0f : 3.0f * profile_scale));
        assert(s_profile_record.small_peak == (speed_profile ? 60.0f : 2.0f * profile_scale));
        assert(s_profile_record.big_travel == (speed_profile ? 0.0f : 25.0f));
        assert(s_profile_record.small_travel == 20.0f);
        assert(s_profile_record.heading_travel == (speed_profile ? 0.0f : 20.0f));
        assert(s_profile_record.reverse == phase_set);
        for (i = 0U; i < samples; ++i) {
            uint32_t drive_ms = i < pre / 4U ? 0U : i * 4U - pre;
            int active = i >= pre / 4U && i < (pre + drive) / 4U;
            int big = active ? (int)roundf((speed_profile ? YawIdent_SpeedWave(drive_ms, 0U, phase_set) :
                (cd ? YawIdent_CDWave(drive_ms, 0U, phase_set) :
                 YawIdent_DualWave(drive_ms, 0U, phase_set))) * 100.0f) : 0;
            int small = active ? (int)roundf((speed_profile ? YawIdent_SpeedWave(drive_ms, 1U, phase_set) :
                (cd ? YawIdent_CDWave(drive_ms, 1U, phase_set) :
                 YawIdent_DualWave(drive_ms, 1U, phase_set))) * 100.0f) : 0;
            assert(abs(streamed[i].big_rpm - big) <= (speed_profile ? 1 : 0));
            assert(abs(streamed[i].small_rpm - small) <= (speed_profile ? 1 : 0));
            assert(abs(streamed[i].offset_cdeg - big - small) <= (speed_profile ? 2 : 0));
        }
    } else {
        int sign = op == YAW_IDENT_ARM_SLOW_REVERSE ? -1 : 1;
        assert(streamed[1375].offset_cdeg == sign * (int)amplitude);
        assert(streamed[3125].offset_cdeg == -sign * (int)amplitude);
    }
    assert(s_owned); /* Completing and receiving every record does not release. */
    remote.remote.s[1] = Remote_SWITCH_UP; cycle(clock_ms + 1U);
    command(YAW_IDENT_RELEASE, id, 0U); cycle(clock_ms + 1U);
    assert(!s_owned);
}

static void stream_integration(void) {
    unsigned mode, i;
    setup();
    stream_request(YAW_IDENT_ARM_DUAL_A, 1U, 3U, 0U); cycle(clock_ms);
    assert(s_owned && s_state.phase == YAW_IDENT_ARMED && !yaw.output_state);
    setup();
    stream_request(YAW_IDENT_ARM_SLOW, 1U, 1U, 1500U); cycle(clock_ms);
    assert(s_owned && s_state.phase == YAW_IDENT_ARMED && !yaw.output_state);
    setup(); fresh(0xfffffff0U);
    if (stream_fixture_path) {
        stream_wire_fixture = fopen(stream_fixture_path, "wb");
        assert(stream_wire_fixture);
    }
    stream_complete(YAW_IDENT_BENCH, 0U, 0U);
    if (stream_wire_fixture) { fclose(stream_wire_fixture); stream_wire_fixture = NULL; }
    stream_complete(YAW_IDENT_ARM_SLOW, 1U, 1500U);
    stream_complete(YAW_IDENT_ARM_SLOW_REVERSE, 2U, 1000U);
    if (dual_stream_fixture_path) {
        stream_wire_fixture = fopen(dual_stream_fixture_path, "wb");
        assert(stream_wire_fixture);
    }
    stream_complete(YAW_IDENT_ARM_DUAL_A, 3U, 0U);
    if (stream_wire_fixture) { fclose(stream_wire_fixture); stream_wire_fixture = NULL; }
    stream_complete(YAW_IDENT_ARM_DUAL_B, 3U, 0U);
    if (cd_stream_fixture_path) {
        stream_wire_fixture = fopen(cd_stream_fixture_path, "wb");
        assert(stream_wire_fixture);
    }
    stream_complete(YAW_IDENT_ARM_CD_C, 4U, 0U);
    if (stream_wire_fixture) { fclose(stream_wire_fixture); stream_wire_fixture = NULL; }
    stream_complete(YAW_IDENT_ARM_CD_D, 4U, 0U);
    if (e_stream_fixture_path) {
        stream_wire_fixture = fopen(e_stream_fixture_path, "wb");
        assert(stream_wire_fixture);
    }
    stream_complete(YAW_IDENT_ARM_CD_E, 4U, 0U);
    if (stream_wire_fixture) { fclose(stream_wire_fixture); stream_wire_fixture = NULL; }
    if (speed_stream_fixture_path) {
        stream_wire_fixture = fopen(speed_stream_fixture_path, "wb");
        assert(stream_wire_fixture);
    }
    stream_complete(YAW_IDENT_ARM_SPEED_S1, 5U, 0U);
    if (stream_wire_fixture) { fclose(stream_wire_fixture); stream_wire_fixture = NULL; }
    stream_complete(YAW_IDENT_ARM_SPEED_S2, 5U, 0U);
    if (speed_s3_stream_fixture_path) {
        stream_wire_fixture = fopen(speed_s3_stream_fixture_path, "wb");
        assert(stream_wire_fixture);
    }
    stream_complete(YAW_IDENT_ARM_SPEED_S3, 5U, 0U);
    if (stream_wire_fixture) { fclose(stream_wire_fixture); stream_wire_fixture = NULL; }
    setup(); trace_dma_paced = 1U; trace_dma_ready_ms = clock_ms;
    stream_complete(YAW_IDENT_BENCH, 0U, 0U);
    trace_dma_paced = 0U;

    for (mode = 0U; mode < 7U; ++mode) {
        setup();
        stream_request(YAW_IDENT_BENCH, 1U, 0U, 0U); cycle(clock_ms);
        assert(s_recording && !yaw.output_state && !pitch.output_state);
        if (mode == 0U) remote.remote.s[1] = Remote_SWITCH_DOWN;
        if (mode == 1U) remote.remote.ch[0] = 11;
        if (mode == 2U) tx_ready = 0U;
        if (mode == 3U) { s_count = CAN_TRACE_CAPACITY - 1U; s_send_index = 0U; } /* Last slot reserved for abort. */
        if (mode == 4U) observation.gyro[1] = NAN;
        for (i = 1U; i <= 330U; ++i) {
            if (i % 50U == 0U) command(YAW_IDENT_KEEPALIVE, 1U, 0U);
            if (mode == 5U) stream_request(YAW_IDENT_STREAM_ACK, 999U, 0U, 5001U);
            if (mode == 6U) stream_request(YAW_IDENT_STREAM_ACK, 1U, 0U, 5001U);
            cycle(clock_ms + 1U);
            assert(!yaw.output_state && !pitch.output_state);
            if (s_state.phase == YAW_IDENT_ABORTED) break;
        }
        assert(s_state.phase == YAW_IDENT_ABORTED);
        assert(s_meta.phase == YAW_IDENT_ABORTED && !s_recording);
        if (mode == 2U || mode >= 5U) assert(s_state.reason == YAW_IDENT_STREAM_STALLED);
        if (mode == 3U) assert(s_state.reason == YAW_IDENT_BUFFER_FULL && s_count == CAN_TRACE_CAPACITY);
        assert(s_ack_index == 0U && s_owned);
    }
    setup();
    stream_request(YAW_IDENT_BENCH, 1U, 0U, 0U); cycle(clock_ms);
    cycle(clock_ms + 5U);
    assert(!s_stream_timing_bad && !(YawIdent_StorageData.records[1].flags & (1U << 11)));
    s_state.phase = YAW_IDENT_DONE; s_count = s_send_index = 5001U;
    stream_request(YAW_IDENT_STREAM_ACK, 1U, 0U, 5001U); cycle(clock_ms + 1U);
    assert(!s_stream_timing_bad); /* Millisecond quantization jitter within 10 ms is valid. */

    setup();
    stream_request(YAW_IDENT_BENCH, 1U, 0U, 0U); cycle(clock_ms);
    s_state.phase = YAW_IDENT_DONE; s_count = s_send_index = 5001U; s_stream_timing_bad = 1U;
    stream_request(YAW_IDENT_STREAM_ACK, 1U, 0U, 5001U); cycle(clock_ms + 1U);
    assert(s_stream_timing_bad); /* A complete but irregular optional BENCH is rejected by the host. */

    for (mode = 1U; mode <= 4U; ++mode) {
        setup();
        stream_request(YAW_IDENT_BENCH, 1U, 0U, 0U); cycle(clock_ms);
        trace_fixture_fault = (uint8_t)mode;
        cycle(clock_ms + 4U);
        assert(s_stream_timing_bad); /* Missing evidence/faults latch until the next capture. */
        s_state.phase = YAW_IDENT_DONE; s_count = s_send_index = 5001U;
        stream_request(YAW_IDENT_STREAM_ACK, 1U, 0U, 5001U); cycle(clock_ms + 1U);
        assert(s_stream_timing_bad);
    }

    setup();
    stream_request(YAW_IDENT_ARM_SLOW, 1U, 1U, 1500U); cycle(clock_ms);
    start();
    Motor_Big_YawMotor.output = 0.5f; Motor_Small_YawMotor.output = 0.25f;
    for (i = 1U; i <= 330U; ++i) {
        if (i % 50U == 0U) command(YAW_IDENT_KEEPALIVE, 1U, 0U);
        cycle(clock_ms + 1U); /* Heartbeat alone must not sustain motion. */
        if (s_state.phase == YAW_IDENT_ABORTED) break;
    }
    assert(s_state.reason == YAW_IDENT_STREAM_STALLED && s_owned);
    assert(!yaw.output_state && !pitch.output_state);
    assert(Motor_Big_YawMotor.output == 0.0f && Motor_Small_YawMotor.output == 0.0f);
}

static void frozen_feedback_failure(void) {
    unsigned kind;
    for (kind = 0U; kind < 4U; ++kind) {
        YawIdent_Record *r;
        setup(); arm(1U); start(); fresh(clock_ms + 2U);
        if (kind == 0U) ready = 0U;
        if (kind == 1U) Motor_Big_YawMotor.is_online = 0U;
        if (kind == 2U) Motor_Small_YawMotor.is_online = 0U;
        if (kind == 3U) observation.tick_ms = clock_ms - 11U;
        YawIdentApp_Pre();
        assert(s_state.reason == YAW_IDENT_FEEDBACK_BAD && s_feedback_failure_latched);
        ready = Motor_Big_YawMotor.is_online = Motor_Small_YawMotor.is_online = 1U;
        observation.tick_ms = clock_ms;
        YawIdentApp_Post();
        r = &YawIdent_StorageData.records[s_count - 1U];
        assert(r->phase == YAW_IDENT_ABORTED && !r->big_command && !r->small_command);
        if (kind < 3U) assert(r->flags & (1U << (12U + kind)));
        else assert(r->imu_age_ms == 11U);
    }
}

int main(int argc, char **argv) {
    stream_fixture_path = argc > 3 ? argv[3] : NULL;
    dual_stream_fixture_path = argc > 4 ? argv[4] : NULL;
    cd_stream_fixture_path = argc > 5 ? argv[5] : NULL;
    e_stream_fixture_path = argc > 6 ? argv[6] : NULL;
    speed_stream_fixture_path = argc > 7 ? argv[7] : NULL;
    speed_s3_stream_fixture_path = argc > 8 ? argv[8] : NULL;
    FILE *fixture = argc > 1 ? fopen(argv[1], "wb") : NULL;
    if (argc > 1) assert(fixture);
    full_trial(1U, 100U, fixture);
    if (fixture) fclose(fixture);
    full_trial(2U, 0xfffffffcU, NULL);
    failures();
    remote_diagnostics(argc > 2 ? argv[2] : NULL);
    wheel_safety();
    switch_transition();
    pitch_configuration();
    rate_output_guard();
    stream_integration();
    frozen_feedback_failure();
    puts("Production identification supervisor: all tests passed.");
    return 0;
}
