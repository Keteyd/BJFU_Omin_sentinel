#include "app_yaw_identification.h"
#include "module_yaw_identification.h"
#include "app_ins.h"
#include "module_gimbal.h"
#include "module_big_yaw_tune.h"
#include "module_shoot.h"
#include "periph_pc_comm.h"
#include "periph_remote.h"
#include "periph_motor.h"
#include "periph_DMmotor.h"
#include "sys_const.h"
#include "sys_robot_actuators.h"
#include "util_can.h"

/* Deployment builds keep the read-only probe/release protocol but reject all
 * excitation requests. Native identification tests explicitly enable them. */
#ifndef YAW_IDENT_MOTION_ENABLED
#define YAW_IDENT_MOTION_ENABLED 0U
#endif

YawIdent_Storage YawIdent_StorageData;
typedef char IdentStorageFits[(sizeof(YawIdent_Storage) == sizeof(YawBurst_Buffer)) ? 1 : -1];
typedef char IdentCountMatches[(YAW_IDENT_SAMPLES == YAW_IDENT_WIRE_SAMPLES) ? 1 : -1];

typedef struct { YawIdent_Packet packet; uint32_t tick; uint8_t safe; } Ident_Message;
static Ident_Message s_queue[8];
static volatile uint8_t s_head, s_tail, s_overflow, s_owned, s_cancel;
static volatile uint32_t s_seen_time;
static volatile uint8_t s_seen;
static YawIdent_State s_state;
static YawIdent_Result s_ref;
static YawIdent_Metadata s_meta;
static uint16_t s_count, s_send_index;
static uint8_t s_meta_part, s_part, s_download, s_ack_pending, s_recording;
static uint8_t s_info_pending;
static YawIdent_RemoteRecord s_remote_record;
static uint16_t s_remote_sequence;
static uint8_t s_remote_pending, s_remote_part;
static uint8_t s_ack[12];
static uint32_t s_last_sample, s_status_time;
static float s_pitch_anchor;
static YawIdent_ProfileRecord s_profile_record;
static uint16_t s_ack_index;
static uint32_t s_stream_progress_ms;
static uint8_t s_profile_part, s_stream_timing_bad;
static YawIdent_Observation s_feedback_failure;
static uint8_t s_feedback_failure_latched;

static uint8_t Ident_StreamHealthy(void)
{
    return (uint8_t)(!s_state.request.profile ||
        ((uint16_t)(s_count - s_ack_index) < CAN_TRACE_CAPACITY - 1U &&
         (s_count == s_ack_index || HAL_GetTick() - s_stream_progress_ms <= YAW_IDENT_STREAM_TIMEOUT_MS)));
}

uint8_t YawIdentApp_OwnsControl(void) { return s_owned; }
uint8_t YawIdentApp_IsOnline(void)
{
    return (uint8_t)(s_owned || (s_seen && HAL_GetTick() - s_seen_time <= 750U));
}

static uint8_t Ident_Neutral(const Remote_RemoteDataTypeDef *r)
{
    return (uint8_t)(YawIdent_NeutralMask(r->remote.ch, r->mouse.x, r->mouse.y,
                                       r->mouse.l, r->mouse.r) == 0U);
}

static void Ident_Observe(YawIdent_Observation *o, INS_Observation *imu)
{
    uint32_t mask = __get_PRIMASK();
    const Remote_RemoteDataTypeDef *r = Remote_GetRemoteDataPtr();
    __disable_irq();
    memset(o, 0, sizeof(*o));
    INS_ReadObservation(imu);
    o->now_ms = HAL_GetTick();
    o->big_deg = Motor_Big_YawMotor.encoder.limited_angle;
    o->imu_yaw_deg = imu->yaw_deg;
    o->big_rate_dps = Motor_Big_YawMotor.encoder.speed * 6.0f;
    o->small_rate_dps = Motor_Small_YawMotor.encoder.speed * 6.0f;
    o->imu_age_ms = YawCapture_Age(o->now_ms, imu->tick_ms);
    o->big_age_ms = YawCapture_Age(o->now_ms, Motor_Big_YawMotor.last_update_time);
    o->small_age_ms = YawCapture_Age(o->now_ms, Motor_Small_YawMotor.last_update_time);
    o->remote_age_ms = YawCapture_Age(o->now_ms, r->last_update_time);
    o->remote_up = (uint8_t)(r->state == Remote_STATE_CONNECTED && r->remote.s[1] == Remote_SWITCH_UP);
    o->remote_down = (uint8_t)(r->state == Remote_STATE_CONNECTED && r->remote.s[1] == Remote_SWITCH_DOWN);
    o->remote_middle = (uint8_t)(r->state == Remote_STATE_CONNECTED && r->remote.s[1] == Remote_SWITCH_MIDDLE);
    o->operator_neutral = Ident_Neutral(r);
    o->yaw_outputs_off = (uint8_t)(GimbalYaw_GetGimbalYawPtr()->output_state == 0U &&
        Motor_Big_YawMotor.output == 0.0f && Motor_Small_YawMotor.output == 0.0f);
    if (!INS_IsReady()) o->feedback_faults |= 1U << 12;
    if (!Motor_Big_YawMotor.is_online) o->feedback_faults |= 1U << 13;
    if (!Motor_Small_YawMotor.is_online) o->feedback_faults |= 1U << 14;
    if (!(isfinite(imu->roll_deg) && isfinite(imu->pitch_deg) &&
        isfinite(imu->gyro[0]) && isfinite(imu->gyro[1]) && isfinite(imu->gyro[2]) &&
        isfinite(o->big_deg) && o->big_deg >= 0.0f && o->big_deg < 360.0f &&
        isfinite(o->imu_yaw_deg) && isfinite(o->big_rate_dps) && isfinite(o->small_rate_dps) &&
        GimbalYaw_DiagSmallEnable && SMALL_YAW_ANGLE_OPEN_TEST == 0U && BIG_YAW_PASSIVE_SMALL_TEST == 0U &&
        YawLimits_Position(Motor_Small_YawMotor.encoder.limited_angle, &o->small_joint_deg)))
        o->feedback_faults |= 1U << 15;
    o->feedback_valid = (uint8_t)(o->feedback_faults == 0U);
    o->capture_ready = 1U;
    o->capture_capacity = YAW_IDENT_SAMPLES;
    __set_PRIMASK(mask);
}

static void Ident_Config(float v[32])
{
    float pitch_kp, pitch_kd;
    RobotActuators_GetPitchMitGains(&pitch_kp, &pitch_kd);
    float values[32] = {
        GimbalYaw_TuneBigAngKp, GimbalYaw_TuneBigSpdKp, GimbalYaw_TuneBigEffortLimit,
        GimbalYaw_TuneBigSpeedFilterTauS, GimbalYaw_TuneBigAngKi, GimbalYaw_TuneBigAngKd,
        GimbalYaw_TuneBigSpdKi, GimbalYaw_TuneBigSpdKd,
        GimbalYaw_TuneSmallAngKp, GimbalYaw_TuneSmallSpdKp, GimbalYaw_DiagEffortLimit,
        GimbalYaw_TuneSmallSpeedFilterAlpha, GimbalYaw_TuneSmallAngKi, GimbalYaw_TuneSmallAngKd,
        GimbalYaw_TuneSmallSpdKi, GimbalYaw_TuneSmallSpdKd,
        GimbalYaw_TuneSmallSpeedLimitRpm, GimbalYaw_TuneSmallManualMaxStepDeg,
        GimbalYaw_DiagSmallDirection, GimbalYaw_DiagBigDirection,
        GimbalPitch_TuneImuKp, GimbalPitch_TuneImuKd, GimbalPitch_TuneGravityEffort,
        GimbalPitch_TuneRateLimitRadS, GimbalPitch_TuneAngleDeadbandRad,
        GimbalPitch_TuneRateDeadbandRadS, (float)GimbalPitch_ImuEnable,
        pitch_kp, pitch_kd,
        (float)GimbalYaw_GetGimbalYawPtr()->control_state,
        (float)GimbalPitch_GetGimbalPitchPtr()->control_state, (float)GimbalYaw_DiagSmallEnable};
    memcpy(v, values, sizeof(values));
}

static uint8_t Ident_ConfigValid(const float v[32])
{
    unsigned i;
    for (i = 0; i < 32U; ++i) if (!isfinite(v[i])) return 0U;
    return (uint8_t)(v[2] > 0.0f && v[2] <= 30.0f && v[10] > 0.0f && v[10] <= 6.0f &&
        v[19] == 1.0f && (v[18] == 1.0f || v[18] == -1.0f) &&
        v[29] == 1.0f && v[30] == 1.0f && v[31] == 1.0f && v[26] == 1.0f);
}

void YawIdentApp_Receive(const uint8_t payload[12])
{
    YawIdent_Packet p;
    uint8_t next;
    if (!YawIdent_DecodeRequest(payload, &p)) return;
    s_seen = 1U; s_seen_time = HAL_GetTick();
    if (p.op == YAW_IDENT_CANCEL && s_owned && p.id == s_state.request.trial_id) s_cancel = 1U;
    next = (uint8_t)((s_head + 1U) % 8U);
    if (next == s_tail) { s_overflow = 1U; return; }
    s_queue[s_head].packet = p;
    s_queue[s_head].tick = HAL_GetTick();
    s_queue[s_head].safe = PC_Comm_IsRemoteSafe();
    __DMB();
    s_head = next;
}

void YawIdentApp_Stop(uint8_t reason)
{
    if (!s_owned) return;
    if (s_state.phase < YAW_IDENT_DONE) YawIdent_Abort(&s_state, (YawIdent_Reason)reason);
    s_ref.reference_valid = 0U;
    GimbalYaw_SetGimbalYawOutputState(0U);
    GimbalPitch_SetGimbalPitchOutputState(0U);
}

uint8_t YawIdentApp_AllowYaw(void)
{
    YawIdent_Observation o;
    INS_Observation imu;
    if (!s_owned || !s_ref.reference_valid || s_cancel || s_overflow || !Ident_StreamHealthy()) return 0U;
    Ident_Observe(&o, &imu);
    return (uint8_t)(o.remote_down && !o.remote_up && o.remote_age_ms <= 50U &&
        o.operator_neutral && YawIdent_FeedbackValid(&o) && YawIdent_InsideTravel(&o) &&
        YawIdent_RelativeTravel(&s_state, &o) &&
        o.now_ms - s_state.previous_ms <= YAW_IDENT_MAX_STEP_MS &&
        o.now_ms - s_state.heartbeat_ms <= YAW_IDENT_HEARTBEAT_MS);
}

float YawIdentApp_BigReference(void)
{
    return s_ref.big_anchor_deg + s_ref.big_offset_deg;
}

uint8_t YawIdentApp_UsesSpeedReference(void)
{
    return (uint8_t)(s_owned && s_state.request.profile == YAW_IDENT_PROFILE_SPEED);
}

float YawIdentApp_BigSpeedReferenceRpm(void)
{
    return YawIdentApp_UsesSpeedReference() ? s_ref.big_offset_deg / 6.0f : 0.0f;
}

float YawIdentApp_SmallSpeedReferenceRpm(void)
{
    return YawIdentApp_UsesSpeedReference() ? s_ref.small_heading_offset_deg / 6.0f : 0.0f;
}

void YawIdentApp_ValidateOutput(void)
{
    YawIdent_Observation o;
    INS_Observation imu;
    YawIdent_Reason reason = YAW_IDENT_OK;
    uint32_t now;
    if (!s_owned || !s_ref.reference_valid) return;
    Ident_Observe(&o, &imu);
    now = o.now_ms;
    if (s_cancel || s_overflow) reason = YAW_IDENT_OPERATOR_STOP;
    else if (!Ident_StreamHealthy())
        reason = (uint16_t)(s_count - s_ack_index) >= CAN_TRACE_CAPACITY - 1U ?
            YAW_IDENT_BUFFER_FULL : YAW_IDENT_STREAM_STALLED;
    else if (o.remote_age_ms > 50U || !o.remote_down || o.remote_up || o.remote_middle ||
             !o.operator_neutral)
        reason = YAW_IDENT_OPERATOR_STOP;
    else if (!YawIdent_FeedbackValid(&o)) reason = YAW_IDENT_FEEDBACK_BAD;
    else if (!YawIdent_InsideTravel(&o) || !YawIdent_RelativeTravel(&s_state, &o))
        reason = YAW_IDENT_TRAVEL;
    else if (now - s_state.previous_ms > YAW_IDENT_MAX_STEP_MS) reason = YAW_IDENT_TIMING;
    else if (now - s_state.heartbeat_ms > YAW_IDENT_HEARTBEAT_MS) reason = YAW_IDENT_LINK_LOST;
    else if (!GimbalYaw_DiagBigActive) reason = YAW_IDENT_YAW_CONTROL_INACTIVE;
    else if (!isfinite(Motor_Big_YawMotor.output) || !isfinite(Motor_Small_YawMotor.output))
        reason = YAW_IDENT_OUTPUT_NONFINITE;
    else if (now - s_state.started_ms >= YAW_IDENT_PRE_MS && !GimbalPitch_DiagMotorOnline)
        reason = YAW_IDENT_PITCH_OFFLINE;
    if (reason != YAW_IDENT_OK) YawIdentApp_Stop(reason);
}

static void Ident_Ack(const YawIdent_Packet *p, uint8_t status)
{
    uint32_t now = HAL_GetTick();
    memcpy(s_ack, &p->id, 4U);
    s_ack[4] = p->op; s_ack[5] = status;
    s_ack[6] = (uint8_t)s_state.phase; s_ack[7] = (uint8_t)s_state.reason;
    memcpy(s_ack + 8U, &now, 4U);
    s_ack_pending = 1U;
}

static void Ident_Request(const Ident_Message *m, const YawIdent_Observation *o)
{
    const YawIdent_Packet *p = &m->packet;
    uint8_t status = YAW_IDENT_REJECTED;
    if (p->op == YAW_IDENT_PROBE) {
        s_info_pending = 1U;
        if (!s_owned && !s_remote_pending) {
            /* Called in Pre's critical section. Freeze all three fragments. */
            const Remote_RemoteDataTypeDef *r = Remote_GetRemoteDataPtr();
            s_remote_record.tick_ms = o->now_ms;
            memcpy(s_remote_record.channels, r->remote.ch, sizeof(s_remote_record.channels));
            s_remote_record.mouse_x = r->mouse.x; s_remote_record.mouse_y = r->mouse.y;
            s_remote_record.mouse_left = r->mouse.l; s_remote_record.mouse_right = r->mouse.r;
            s_remote_record.age_ms = o->remote_age_ms;
            s_remote_record.failed_mask = YawIdent_NeutralMask(r->remote.ch, r->mouse.x,
                r->mouse.y, r->mouse.l, r->mouse.r);
            ++s_remote_sequence;
            s_remote_part = 0U; s_remote_pending = 1U;
        }
        return;
    }
    if (p->op == YAW_IDENT_KEEPALIVE) {
        if (s_owned) (void)YawIdent_Heartbeat(&s_state, p->id, m->tick);
        return;
    }
    if (p->op == YAW_IDENT_STREAM_ACK) {
        /* Cumulative contiguous receipt, not just a live UART or HAL transmit success. */
        if (s_owned && s_state.request.profile && p->id == s_state.request.trial_id &&
            o->now_ms - m->tick <= 100U && p->amplitude_cdeg > s_ack_index &&
            p->amplitude_cdeg <= s_send_index) {
            s_ack_index = p->amplitude_cdeg;
            s_stream_progress_ms = o->now_ms;
        }
        return;
    }
    if (p->op == YAW_IDENT_ARM || p->op == YAW_IDENT_ARM_SLOW ||
        p->op == YAW_IDENT_ARM_SLOW_REVERSE || p->op == YAW_IDENT_BENCH ||
        p->op == YAW_IDENT_ARM_DUAL_A || p->op == YAW_IDENT_ARM_DUAL_B ||
        p->op == YAW_IDENT_ARM_CD_C || p->op == YAW_IDENT_ARM_CD_D ||
        p->op == YAW_IDENT_ARM_CD_E || p->op == YAW_IDENT_ARM_SPEED_S1 ||
        p->op == YAW_IDENT_ARM_SPEED_S2 || p->op == YAW_IDENT_ARM_SPEED_S3) {
#if YAW_IDENT_MOTION_ENABLED == 0U
        Ident_Ack(p, YAW_IDENT_REJECTED);
        return;
#endif
        YawIdent_Request r = {0};
        float config[32];
        Ident_Config(config);
        r.trial_id = p->id; r.axis = p->axis; r.amplitude_deg = p->amplitude_cdeg * .01f;
        r.profile = p->op == YAW_IDENT_ARM ? YAW_IDENT_PROFILE_SHORT :
            (p->op == YAW_IDENT_BENCH ? YAW_IDENT_PROFILE_BENCH :
             (p->op == YAW_IDENT_ARM_DUAL_A || p->op == YAW_IDENT_ARM_DUAL_B ?
              YAW_IDENT_PROFILE_DUAL :
              (p->op == YAW_IDENT_ARM_CD_C || p->op == YAW_IDENT_ARM_CD_D ||
               p->op == YAW_IDENT_ARM_CD_E ? YAW_IDENT_PROFILE_CD :
               (p->op == YAW_IDENT_ARM_SPEED_S1 || p->op == YAW_IDENT_ARM_SPEED_S2 ||
                p->op == YAW_IDENT_ARM_SPEED_S3 ?
                YAW_IDENT_PROFILE_SPEED : YAW_IDENT_PROFILE_SLOW))));
        r.reverse = (p->op == YAW_IDENT_ARM_CD_E || p->op == YAW_IDENT_ARM_SPEED_S3) ? 2U :
            (uint8_t)(p->op == YAW_IDENT_ARM_SLOW_REVERSE ||
                p->op == YAW_IDENT_ARM_DUAL_B || p->op == YAW_IDENT_ARM_CD_D ||
                p->op == YAW_IDENT_ARM_SPEED_S2);
        if (s_owned || PC_Comm_IsGimbalTuneSessionOnline()) status = YAW_IDENT_BUSY;
        else if (!s_overflow && Ident_ConfigValid(config) && YawIdent_Arm(&s_state, &r, o)) {
            s_owned = 1U; s_cancel = 0U; s_count = 0U; s_download = 0U; s_recording = 0U;
            s_ack_index = s_send_index = 0U; s_meta_part = s_part = s_profile_part = 0U;
            s_stream_timing_bad = 0U;
            s_feedback_failure_latched = 0U;
            s_stream_progress_ms = o->now_ms;
            s_remote_pending = 0U;
            memset(&s_meta, 0, sizeof(s_meta));
            s_meta.id = p->id; s_meta.axis = p->axis;
            s_meta.version = r.profile ? YAW_IDENT_STREAM_VERSION : 1U;
            s_meta.build = YAW_IDENT_BUILD; s_meta.period_ms = YAW_IDENT_PERIOD_MS; s_meta.setup = 1U;
            s_meta.values[0] = r.amplitude_deg;
            s_meta.values[7] = YAW_LIMIT_ZERO_DEG;
            s_meta.values[8] = YAW_LIMIT_MIN_DEG; s_meta.values[9] = YAW_LIMIT_MAX_DEG;
            memcpy(s_meta.values + 10, config, sizeof(config));
            memset(&s_profile_record, 0, sizeof(s_profile_record));
            s_profile_record.id = p->id; s_profile_record.build = YAW_IDENT_BUILD;
            s_profile_record.baud = 460800U;
            s_profile_record.samples = YawIdent_Samples(&r);
            s_profile_record.period_ms = YAW_IDENT_PERIOD_MS;
            s_profile_record.capacity = CAN_TRACE_CAPACITY;
            s_profile_record.duration_ms = (uint16_t)YawIdent_Duration(&r);
            {
                const uint16_t slow_knots[8] = {0U, 2000U, 4000U, 7000U, 11000U, 14000U, 16000U, 20000U};
                const uint16_t dual_knots[8] = {0U, 2000U, 16000U, 20000U, 0U, 0U, 0U, 0U};
                const uint16_t cd_knots[8] = {0U, 3000U, 9000U, 31000U, 36000U, 0U, 0U, 0U};
                const uint16_t speed_knots[8] = {0U, 3000U, 31000U, 36000U, 0U, 0U, 0U, 0U};
                const uint16_t *knots = r.profile == YAW_IDENT_PROFILE_SPEED ? speed_knots :
                    (r.profile == YAW_IDENT_PROFILE_CD ? cd_knots :
                    (r.profile == YAW_IDENT_PROFILE_DUAL ? dual_knots : slow_knots));
                memcpy(s_profile_record.knots_ms, knots, sizeof(s_profile_record.knots_ms));
            }
            if (r.profile == YAW_IDENT_PROFILE_SPEED) {
                s_profile_record.big_peak = YAW_IDENT_SPEED_BIG_PEAK_DPS;
                s_profile_record.small_peak = YAW_IDENT_SPEED_SMALL_PEAK_DPS;
            } else if (r.profile == YAW_IDENT_PROFILE_CD) {
                const float ladder_scale[3] = {1.0f, 4.0f, 5.0f};
                s_profile_record.big_peak = YAW_IDENT_DUAL_BIG_PEAK_DEG * ladder_scale[r.reverse];
                s_profile_record.small_peak = YAW_IDENT_DUAL_SMALL_PEAK_DEG * ladder_scale[r.reverse];
            } else {
                s_profile_record.big_peak = r.profile == YAW_IDENT_PROFILE_DUAL ?
                    YAW_IDENT_DUAL_BIG_PEAK_DEG : YAW_IDENT_SLOW_BIG_PEAK_DEG;
                s_profile_record.small_peak = r.profile == YAW_IDENT_PROFILE_DUAL ?
                    YAW_IDENT_DUAL_SMALL_PEAK_DEG : YAW_IDENT_SLOW_SMALL_PEAK_DEG;
            }
            s_profile_record.big_travel = r.profile == YAW_IDENT_PROFILE_SPEED ?
                0.0f : YAW_IDENT_SLOW_BIG_TRAVEL_DEG;
            s_profile_record.small_travel = YAW_IDENT_SLOW_SMALL_TRAVEL_DEG;
            s_profile_record.heading_travel = r.profile == YAW_IDENT_PROFILE_SPEED ?
                0.0f : YAW_IDENT_SLOW_HEADING_TRAVEL_DEG;
            s_profile_record.center_deg = YAW_IDENT_SLOW_CENTER_DEG;
            s_profile_record.profile = r.profile; s_profile_record.reverse = r.reverse;
            s_profile_record.version = YAW_IDENT_STREAM_VERSION;
            GimbalYaw_TuneSmallStepDeg = 0.0f; GimbalPitch_TuneStepRad = 0.0f;
            status = YAW_IDENT_ACCEPTED;
        }
    } else if (s_owned && p->id == s_state.request.trial_id) {
        if (p->op == YAW_IDENT_CANCEL) {
            YawIdentApp_Stop(YAW_IDENT_OPERATOR_STOP);
            status = YAW_IDENT_ACCEPTED;
        } else if (s_state.phase >= YAW_IDENT_DONE && m->safe && o->remote_up && o->yaw_outputs_off &&
                   o->remote_age_ms <= 50U && o->now_ms - m->tick <= 100U) {
            if (p->op == YAW_IDENT_DOWNLOAD) {
                s_meta.count = s_count; s_meta.phase = (uint8_t)s_state.phase;
                s_meta.reason = (uint8_t)s_state.reason;
                s_download = 1U; s_send_index = s_state.request.profile ? s_ack_index : 0U;
                s_meta_part = 0U; s_profile_part = 0U; s_part = 0U;
                status = YAW_IDENT_ACCEPTED;
            } else if (p->op == YAW_IDENT_RELEASE) {
                Can_TraceEnd();
                s_download = 0U; s_ref.reference_valid = 0U; s_count = 0U; s_recording = 0U;
                memset(&YawIdent_StorageData, 0, sizeof(YawIdent_StorageData));
                s_state.phase = YAW_IDENT_IDLE;
                s_owned = 0U;
                status = YAW_IDENT_ACCEPTED;
            }
        } else status = YAW_IDENT_UNSAFE;
    }
    Ident_Ack(p, status);
}

void YawIdentApp_Pre(void)
{
    YawIdent_Observation o;
    INS_Observation imu;
    uint8_t old_phase;
    uint32_t mask;
    float config[32];
    /* Validate and publish ownership without an intervening Control_Task write. */
    mask = __get_PRIMASK(); __disable_irq();
    Ident_Observe(&o, &imu);
    while (s_tail != s_head) {
        Ident_Message message = s_queue[s_tail];
        s_tail = (uint8_t)((s_tail + 1U) % 8U);
        Ident_Request(&message, &o);
    }
    if (s_owned && (s_cancel || s_overflow)) YawIdentApp_Stop(YAW_IDENT_OPERATOR_STOP);
    s_overflow = 0U;
    __set_PRIMASK(mask);
    if (!s_owned) return;
    if (s_state.request.profile && s_recording && !Ident_StreamHealthy())
        YawIdentApp_Stop((uint16_t)(s_count - s_ack_index) >= CAN_TRACE_CAPACITY - 1U ?
            YAW_IDENT_BUFFER_FULL : YAW_IDENT_STREAM_STALLED);
    Ident_Config(config);
    if (memcmp(config, s_meta.values + 10, sizeof(config)) != 0)
        YawIdentApp_Stop(YAW_IDENT_CONFIG_CHANGED);
    if (s_recording && (fabsf(imu.roll_deg - s_meta.values[5]) > 5.0f ||
                       fabsf(imu.pitch_deg - s_meta.values[6]) > 5.0f))
        YawIdentApp_Stop(YAW_IDENT_TRAVEL);
    old_phase = (uint8_t)s_state.phase;
    s_ref = YawIdent_Step(&s_state, &o);
    if (old_phase < YAW_IDENT_DONE && s_state.phase == YAW_IDENT_ABORTED &&
        s_state.reason == YAW_IDENT_FEEDBACK_BAD) {
        s_feedback_failure = o;
        s_feedback_failure_latched = 1U;
    }
    if (old_phase == YAW_IDENT_ARMED && s_state.phase == YAW_IDENT_BASELINE) {
        GimbalPitch_GimbalPitchTypeDef *pitch = GimbalPitch_GetGimbalPitchPtr();
        s_pitch_anchor = imu.roll_deg * PI / 180.0f + Const_PITCH_MOTOR_INIT_OFFSETf;
        if (s_state.request.profile != YAW_IDENT_PROFILE_BENCH) {
            pitch->pitch_ref = pitch->pitch_ref_smooth = s_pitch_anchor;
            pitch->motor_ref = GimbalPitch_GetPositionFeedback();
        }
        s_meta.start_ms = o.now_ms;
        s_meta.values[1] = s_ref.big_anchor_deg;
        s_meta.values[2] = s_ref.small_heading_anchor_deg;
        s_meta.values[3] = o.small_joint_deg;
        s_meta.values[4] = s_pitch_anchor;
        s_meta.values[5] = imu.roll_deg; s_meta.values[6] = imu.pitch_deg;
        s_last_sample = o.now_ms - YAW_IDENT_PERIOD_MS;
        s_stream_progress_ms = o.now_ms;
        s_meta.phase = YAW_IDENT_BASELINE;
        s_recording = 1U;
        if (s_state.request.profile) Can_TraceBegin();
    }
    if (s_ref.reference_valid && YawIdentApp_AllowYaw()) {
        GimbalYaw_GetGimbalYawPtr()->yaw_ref = YawIdentApp_UsesSpeedReference() ?
            s_ref.small_heading_anchor_deg : s_ref.small_heading_anchor_deg + s_ref.small_heading_offset_deg;
        GimbalYaw_SetSmallYawRateDps(0.0f);
        GimbalPitch_GetGimbalPitchPtr()->pitch_ref = s_pitch_anchor;
        GimbalYaw_SetGimbalYawOutputState(1U);
        GimbalPitch_SetGimbalPitchOutputState(1U);
    } else {
        GimbalYaw_SetGimbalYawOutputState(0U);
        GimbalPitch_SetGimbalPitchOutputState(0U);
    }
}

static int16_t Ident_I16(float value)
{
    if (!isfinite(value)) return 0;
    return (int16_t)YawLimits_Clamp(roundf(value), -32768.0f, 32767.0f);
}

static uint8_t Ident_Send(uint8_t cmd, const void *data)
{
    uint8_t sent;
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    sent = PC_Comm_SendPacket(cmd, data, 12U);
    __set_PRIMASK(mask);
    return sent;
}

void YawIdentApp_Post(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t payload[12];
    YawIdentApp_ValidateOutput();
    if (s_owned && s_recording && s_state.phase >= YAW_IDENT_DONE) {
        GimbalYaw_Output();
        GimbalPitch_Output();
    }
    if (s_owned && s_recording &&
        s_count < YawIdent_Samples(&s_state.request) &&
        (!s_state.request.profile || (uint16_t)(s_count - s_ack_index) < CAN_TRACE_CAPACITY) &&
        (now - s_last_sample >= YAW_IDENT_PERIOD_MS || s_state.phase >= YAW_IDENT_DONE)) {
        YawIdent_Record *r = s_state.request.profile ?
            &YawIdent_StorageData.trace[s_count % CAN_TRACE_CAPACITY].sample : &YawIdent_StorageData.records[s_count];
        INS_Observation imu;
        YawIdent_Observation o;
        uint32_t mask = __get_PRIMASK(); __disable_irq();
        Ident_Observe(&o, &imu);
        r->tick_ms = o.now_ms; r->yaw_deg = imu.yaw_deg;
        memcpy(r->gyro, imu.gyro, sizeof(r->gyro));
        r->roll_deg = imu.roll_deg; r->pitch_deg = imu.pitch_deg;
        r->big_raw = (uint16_t)Motor_Big_YawMotor.encoder.angle;
        r->small_raw = (uint16_t)Motor_Small_YawMotor.encoder.angle;
        if (s_state.request.profile == YAW_IDENT_PROFILE_SPEED) {
            /* Capture the references actually entering the retained speed
             * PIDs after production soft limiting, in centidegrees/second. */
            r->big_rpm = Ident_I16(GimbalYaw_DiagBigSpeedRefRpm * 600.0f);
            r->small_rpm = Ident_I16(GimbalYaw_DiagSmallSpeedRefRpm * 600.0f);
        } else if (s_state.request.profile) {
            /* Position profiles store centidegrees; speed profile stores
             * centidegrees/second. CAN feedback RPM remains in axes[].rpm. */
            r->big_rpm = Ident_I16(s_ref.big_offset_deg * 100.0f);
            r->small_rpm = Ident_I16(s_ref.small_heading_offset_deg * 100.0f);
        } else {
            r->big_rpm = Ident_I16(o.big_rate_dps / 6.0f);
            r->small_rpm = Ident_I16(o.small_rate_dps / 6.0f);
        }
        r->big_command = isfinite(Motor_Big_YawMotor.output) ?
            (int16_t)YawLimits_Clamp(Motor_Big_YawMotor.output * 1000.0f, -32768.0f, 32767.0f) : 0;
        r->small_command = isfinite(Motor_Small_YawMotor.output) ?
            (int16_t)YawLimits_Clamp(Motor_Small_YawMotor.output * 1000.0f, -32768.0f, 32767.0f) : 0;
        r->offset_cdeg = s_state.request.profile ?
            (int16_t)((int32_t)r->big_rpm + r->small_rpm) :
            Ident_I16((s_ref.big_offset_deg + s_ref.small_heading_offset_deg) * 100.0f);
        r->flags = (uint16_t)(o.feedback_valid | (o.remote_up << 1) | (o.remote_down << 2) |
            (GimbalYaw_DiagBigActive << 3) | (s_ref.reference_valid << 4) |
            ((uint16_t)GimbalYaw_GetGimbalYawPtr()->yaw_ref_limit_status << 5) |
            (GimbalYaw_DiagBigSaturated << 10) |
            (s_count && now - s_last_sample > YAW_IDENT_MAX_STEP_MS ? (1U << 11) : 0U));
        if (s_count && now - s_last_sample > YAW_IDENT_MAX_STEP_MS) s_stream_timing_bad = 1U;
        /* Terminal feedback ages/validity describe the abort observation, not
         * a repaired sensor sample read after shutdown. Other fields stay live. */
        if (s_state.reason == YAW_IDENT_FEEDBACK_BAD && s_feedback_failure_latched) {
            o = s_feedback_failure;
            r->flags = (uint16_t)((r->flags & ~1U) | o.feedback_valid | o.feedback_faults);
        }
        r->imu_age_ms = (uint8_t)(o.imu_age_ms > 255U ? 255U : o.imu_age_ms);
        r->big_age_ms = (uint8_t)(o.big_age_ms > 255U ? 255U : o.big_age_ms);
        r->small_age_ms = (uint8_t)(o.small_age_ms > 255U ? 255U : o.small_age_ms);
        r->phase = (uint8_t)s_state.phase;
        if (s_state.request.profile) {
            YawIdent_TraceRecord *trace = &YawIdent_StorageData.trace[s_count % CAN_TRACE_CAPACITY];
            unsigned axis;
            trace->interval_us = Can_TraceSnapshot(&trace->trace_us, trace->axes);
            if (s_count && (!trace->interval_us ||
                trace->interval_us > YAW_IDENT_MAX_STEP_MS * 1000U)) {
                r->flags |= (1U << 11);
                s_stream_timing_bad = 1U;
            }
            for (axis = 0U; axis < 2U; ++axis) {
                const CanTrace_AxisRecord *a = &trace->axes[axis];
                if (a->flags & 0xfcU || a->failed || a->errors || a->aborted ||
                    (s_count && ((a->flags & 3U) != 3U || a->known_us != trace->interval_us ||
                                  !trace->interval_us ||
                                  trace->interval_us > YAW_IDENT_MAX_STEP_MS * 1000U)) ||
                    (s_state.request.profile == YAW_IDENT_PROFILE_BENCH &&
                     (a->minimum || a->maximum || a->command || a->integral_raw_us)))
                    s_stream_timing_bad = 1U;
            }
        }
        __set_PRIMASK(mask);
        s_last_sample = o.now_ms; ++s_count;
    }
    /* Freeze the actual samples at termination; never pad a short capture. */
    if (s_state.phase >= YAW_IDENT_DONE && (s_recording || s_meta.phase != s_state.phase)) {
        s_recording = 0U;
        Can_TraceEnd();
        s_meta.count = s_count; s_meta.phase = (uint8_t)s_state.phase;
        s_meta.reason = (uint8_t)s_state.reason;
        s_meta_part = 0U;
    }
    if (!YawIdentApp_IsOnline()) return;
    if (s_ack_pending) {
        if (Ident_Send(YAW_IDENT_CMD_ACK, s_ack)) s_ack_pending = 0U;
        return;
    }
    if (now - s_status_time >= 100U) {
        uint32_t id = s_state.last_id;
        YawIdent_Observation o;
        INS_Observation imu;
        float config[32];
        Ident_Observe(&o, &imu);
        Ident_Config(config);
        uint8_t flags = (uint8_t)(s_owned | (YawIdentApp_AllowYaw() << 1) |
            (PC_Comm_IsRemoteSafe() << 2) |
            ((Motor_Big_YawMotor.output == 0.0f && Motor_Small_YawMotor.output == 0.0f) << 3) |
            (YawIdent_FeedbackValid(&o) << 4) | (o.operator_neutral << 5) |
            ((fabsf(o.small_joint_deg) <= YAW_IDENT_SLOW_CENTER_DEG && fabsf(o.big_rate_dps) <= 6.0f &&
              fabsf(o.small_rate_dps) <= 6.0f) << 6) | (Ident_ConfigValid(config) << 7));
        memcpy(payload, &id, 4U); memcpy(payload + 4U, &s_count, 2U);
        payload[6] = (uint8_t)s_state.phase; payload[7] = (uint8_t)s_state.reason;
        payload[8] = flags; payload[9] = s_state.request.axis; payload[10] = 1U;
        payload[11] = (uint8_t)(s_stream_timing_bad | (s_state.request.profile << 1) |
            ((GimbalYaw_DiagBigStopReason & 0x0fU) << 4));
        if (Ident_Send(YAW_IDENT_CMD_STATUS, payload)) s_status_time = now;
        return;
    }
    if (s_info_pending) {
        uint32_t build = YAW_IDENT_BUILD, magic = YAW_IDENT_MAGIC;
        memcpy(payload, &build, 4U); memcpy(payload + 4U, &magic, 4U);
        payload[8] = 1U; payload[9] = 0U; payload[10] = CAN_TRACE_BYTES; payload[11] = 0U;
        if (Ident_Send(YAW_IDENT_CMD_INFO, payload)) s_info_pending = 0U;
        return;
    }
    if (s_remote_pending && !s_owned) {
        YawIdent_Part(&s_remote_record, s_remote_sequence, s_remote_part, payload);
        payload[3] = YAW_IDENT_REMOTE_VERSION;
        if (Ident_Send(YAW_IDENT_CMD_REMOTE, payload) && ++s_remote_part == YAW_IDENT_REMOTE_PARTS)
            s_remote_pending = 0U;
        return;
    }
    if (s_owned && s_state.request.profile && s_state.phase >= YAW_IDENT_BASELINE) {
        if (s_profile_part < YAW_IDENT_PROFILE_PARTS) {
            YawIdent_Part(&s_profile_record, 0U, s_profile_part, payload);
            payload[3] = YAW_IDENT_STREAM_VERSION;
            if (Ident_Send(YAW_IDENT_CMD_PROFILE, payload)) ++s_profile_part;
        } else if (s_meta_part < YAW_IDENT_META_PARTS) {
            YawIdent_Part(&s_meta, s_state.phase >= YAW_IDENT_DONE ? 1U : 0U, s_meta_part, payload);
            payload[3] = YAW_IDENT_STREAM_VERSION;
            if (Ident_Send(YAW_IDENT_CMD_META, payload)) ++s_meta_part;
        } else if (s_send_index < s_count) {
            if (PC_Comm_SendTrace(s_state.request.trial_id, s_send_index + 1U,
                &YawIdent_StorageData.trace[s_send_index % CAN_TRACE_CAPACITY])) ++s_send_index;
        }
        return;
    }
    if (!s_download) return;
    if (!PC_Comm_IsRemoteSafe()) { s_download = 0U; return; }
    if (s_meta_part < YAW_IDENT_META_PARTS) {
        YawIdent_Part(&s_meta, 0U, s_meta_part, payload);
        if (Ident_Send(YAW_IDENT_CMD_META, payload)) ++s_meta_part;
    } else if (s_send_index < s_count) {
        YawIdent_Part(&YawIdent_StorageData.records[s_send_index], s_send_index + 1U, s_part, payload);
        if (Ident_Send(YAW_IDENT_CMD_SAMPLE, payload) && ++s_part == YAW_IDENT_RECORD_PARTS) {
            s_part = 0U; ++s_send_index;
        }
    } else s_download = 0U;
}
