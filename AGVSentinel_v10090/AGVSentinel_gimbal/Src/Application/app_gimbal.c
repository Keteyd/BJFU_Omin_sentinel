#include "app_gimbal.h"
#include "app_attitude_link.h"
#include "app_yaw_identification.h"

#include "app_autoaim.h"
#include "app_ins.h"
#include "module_gimbal.h"
#include "module_yaw_limits.h"
#include "module_big_yaw_tune.h"
#include "module_yaw_peaks.h"
#include "module_shoot.h"
#include "periph_pc_comm.h"
#include "periph_motor.h"
#include "sys_const.h"
#include "sys_robot_actuators.h"
#include "periph_DMmotor.h"
#include "periph_remote.h"
#include "module_yaw_capture.h"
#include "module_yaw_burst.h"
#include "sys_dwt.h"
#include "periph_bmi088.h"

extern volatile float Control_DiagDtS;
static float s_gimbal_dt_s;
#define s_yaw_burst YawIdent_StorageData.legacy

static uint32_t s_pid_tune_sequence;
static uint32_t s_pid_action_sequence;
static uint32_t s_pitch_tune_sequence;
static uint32_t s_pitch_action_sequence;
static uint32_t s_pid_telemetry_time;
static uint8_t s_pid_telemetry_phase;
static uint32_t s_pitch_limits_time;
static uint32_t s_yaw_encoders_time;
static uint32_t s_yaw_limits_time;
static uint32_t s_yaw_coordinator_time;
static uint32_t s_big_yaw_tune_sequence;
static uint32_t s_big_yaw_config_time;
static uint16_t s_big_yaw_request_id;
static uint8_t s_big_yaw_request_status;
static uint32_t s_big_yaw_peaks_time;
static YawPeaks_State s_big_yaw_peaks;
static uint32_t s_big_yaw_full_config_time;
static uint8_t s_big_yaw_full_part;
static uint8_t s_big_yaw_full_snapshot[4][12];

static void GimbalCapture_Fill(YawCapture_Record *record)
{
    INS_Observation imu;
    uint32_t primask = __get_PRIMASK();
    GimbalYaw_GimbalYawTypeDef *yaw = GimbalYaw_GetGimbalYawPtr();
    __disable_irq();
    INS_ReadObservation(&imu);
    record->tick_ms = HAL_GetTick();
    record->imu_sequence = imu.sequence;
    record->imu_age_ms = YawCapture_Age(record->tick_ms, imu.tick_ms);
    record->big_age_ms = YawCapture_Age(record->tick_ms, Motor_Big_YawMotor.last_update_time);
    record->small_age_ms = YawCapture_Age(record->tick_ms, Motor_Small_YawMotor.last_update_time);
    record->rc_yaw = Remote_GetRemoteDataPtr()->remote.ch[2];
    record->flags = (uint16_t)(PC_Comm_IsRemoteSafe() |
        (INS_IsReady() << 1) | (Motor_Big_YawMotor.is_online << 2) |
        (Motor_Small_YawMotor.is_online << 3) | ((yaw->output_state != 0U) << 4) |
        (GimbalYaw_DiagBigActive << 5) | ((GimbalYaw_DiagSmallEnable != 0U) << 6) |
        (BIG_YAW_OUTPUT_INHIBIT_TEST << 7) | (SMALL_YAW_ANGLE_OPEN_TEST << 8) |
        ((uint16_t)yaw->yaw_ref_limit_status << 9));
    record->imu_dt_s = imu.dt_s;
    record->yaw_deg = imu.yaw_deg;
    record->roll_deg = imu.roll_deg;
    record->gyro_x_rad_s = imu.gyro[0];
    record->gyro_y_rad_s = imu.gyro[1];
    record->gyro_z_rad_s = imu.gyro[2];
    record->big_encoder_deg = Motor_Big_YawMotor.encoder.limited_angle;
    record->small_encoder_deg = Motor_Small_YawMotor.encoder.limited_angle;
    record->big_rpm = Motor_Big_YawMotor.encoder.speed;
    record->small_rpm = Motor_Small_YawMotor.encoder.speed;
    record->big_command = Motor_Big_YawMotor.output;
    record->small_command = Motor_Small_YawMotor.output;
    record->yaw_target_deg = yaw->yaw_ref;
    record->big_current_raw = Motor_Big_YawMotor.encoder.current;
    record->small_current_raw = Motor_Small_YawMotor.encoder.current;
    record->pitch_deg = imu.pitch_deg;
    record->small_speed_target_rpm = GimbalYaw_DiagSmallSpeedRefRpm;
    __set_PRIMASK(primask);
}

static void GimbalCapture_Service(void)
{
    static YawCapture_Record record;
    static uint32_t last_sample;
    static uint16_t sequence, skipped;
    static uint8_t part = YAW_CAPTURE_PARTS, online;
    static uint32_t diag_time;
    static uint8_t diag_phase;
    uint32_t now = HAL_GetTick();
    if (YawIdentApp_OwnsControl()) return;
    if (!PC_Comm_IsYawCaptureOnline()) {
        YawBurst_Reset(&s_yaw_burst);
        part = YAW_CAPTURE_PARTS;
        online = 0U;
        return;
    }
    if ((uint32_t)(now - diag_time) >= 250U &&
        (PC_Comm_GetDataPtr()->yaw_capture_requested != 2U ||
         s_yaw_burst.count == YAW_BURST_COUNT)) {
        uint8_t payload[12], sent;
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (diag_phase == 0U) {
            BMI088_ReadDiagTypeDef diag = BMI088_ReadDiag;
            memcpy(payload, &diag, sizeof(payload));
        } else {
            uint32_t read_rejected = INS_DiagReadRejected;
            uint32_t dt_rejected = INS_DiagDtRejected;
            float dt = INS_DiagLastDtS;
            memcpy(payload, &read_rejected, 4U);
            memcpy(payload + 4U, &dt_rejected, 4U);
            memcpy(payload + 8U, &dt, 4U);
        }
        sent = PC_Comm_SendPacket(diag_phase == 0U ? PC_CMD_IMU_READ_DIAG :
            PC_CMD_IMU_TIMING_DIAG, payload, sizeof(payload));
        __set_PRIMASK(primask);
        if (sent) { diag_time = now; diag_phase ^= 1U; }
    }
    if (PC_Comm_GetDataPtr()->yaw_capture_requested == 2U) {
        uint8_t payload[12];
        part = YAW_CAPTURE_PARTS;
        online = 0U;
        if (YawBurst_Due(&s_yaw_burst, now)) {
            YawBurst_Record *sample = &s_yaw_burst.records[s_yaw_burst.count];
            GimbalCapture_Fill(&sample->observation);
            sample->observation.skipped = s_yaw_burst.skipped;
            sample->control_dt_s = Control_DiagDtS;
            sample->gimbal_dt_s = s_gimbal_dt_s;
            ++s_yaw_burst.count;
        }
        if (YawBurst_Payload(&s_yaw_burst, payload)) {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            if (PC_Comm_SendPacket(PC_CMD_YAW_CAPTURE, payload, sizeof(payload)))
                YawBurst_Accept(&s_yaw_burst);
            __set_PRIMASK(primask);
        }
        return;
    }
    YawBurst_Reset(&s_yaw_burst);
    if (!online) {
        last_sample = now - YAW_CAPTURE_PERIOD_MS;
        online = 1U;
    }
    if ((uint32_t)(now - last_sample) >= YAW_CAPTURE_PERIOD_MS) {
        uint32_t periods = (now - last_sample) / YAW_CAPTURE_PERIOD_MS;
        last_sample += periods * YAW_CAPTURE_PERIOD_MS;
        if (part != YAW_CAPTURE_PARTS) {
            skipped = (uint16_t)(skipped + periods);
        } else {
            skipped = (uint16_t)(skipped + periods - 1U);
            GimbalCapture_Fill(&record);
            record.skipped = skipped;
            ++sequence;
            part = 0U;
        }
    }
    if (part < YAW_CAPTURE_PARTS) {
        uint8_t payload[12];
        uint32_t primask;
        YawCapture_Part(&record, sequence, part, payload);
        primask = __get_PRIMASK();
        __disable_irq();
        if (PC_Comm_SendPacket(PC_CMD_YAW_CAPTURE, payload, sizeof(payload))) ++part;
        __set_PRIMASK(primask);
    }
}

static uint8_t GimbalPid_YawOutputsOff(void)
{
    return (uint8_t)(GimbalYaw_GetGimbalYawPtr()->output_state == 0U &&
        Motor_Small_YawMotor.output == 0.0f && Motor_Big_YawMotor.output == 0.0f);
}

static void GimbalPid_ApplyBigYawCommand(void)
{
    uint32_t primask = __get_PRIMASK();
    /* Consume and validate the IRQ mailbox once; never defer a rejected write. */
    __disable_irq();
    if (PC_Comm_Data.big_yaw_tune_sequence != s_big_yaw_tune_sequence) {
        if (PC_Comm_Data.big_yaw_tune_full) {
            s_pid_telemetry_time = 0U;
            s_big_yaw_tune_sequence = PC_Comm_Data.big_yaw_tune_sequence;
            s_big_yaw_request_id = PC_Comm_Data.big_yaw_full_id;
            s_big_yaw_request_status = PC_Comm_Data.big_yaw_full_status;
            if (s_big_yaw_request_status == 1U) {
                s_big_yaw_request_status = BigYaw_TuneRequestStatus(
                    BigYaw_FullValid(PC_BigYawFullValues), PC_Comm_Data.big_yaw_tune_rx_safe,
                    PC_Comm_IsRemoteSafe(), GimbalPid_YawOutputsOff(),
                    HAL_GetTick() - PC_Comm_Data.big_yaw_tune_rx_time);
                if (s_big_yaw_request_status == 1U) {
                    GimbalYaw_GimbalYawTypeDef *yaw = GimbalYaw_GetGimbalYawPtr();
                    GimbalYaw_TuneBigAngKp = PC_BigYawFullValues[0];
                    GimbalYaw_TuneBigSpdKp = PC_BigYawFullValues[1];
                    GimbalYaw_TuneBigEffortLimit = PC_BigYawFullValues[2];
                    GimbalYaw_TuneBigSpeedFilterTauS = PC_BigYawFullValues[3];
                    GimbalYaw_TuneBigAngKi = PC_BigYawFullValues[4];
                    GimbalYaw_TuneBigAngKd = PC_BigYawFullValues[5];
                    GimbalYaw_TuneBigSpdKi = PC_BigYawFullValues[6];
                    GimbalYaw_TuneBigSpdKd = PC_BigYawFullValues[7];
                    BigYaw_ResetPid(&yaw->angPID);
                    BigYaw_ResetPid(&yaw->spdPID);
                }
            }
            __set_PRIMASK(primask);
            return;
        }
        PC_Recv_BigYawTune_t tune = PC_BigYawTune;
        float angle = tune.angle_kp_milli * 0.001f;
        float speed = tune.speed_kp_milli * 0.001f;
        float effort = tune.effort_milli * 0.001f;
        float tau = tune.filter_tau_1e4_s * 0.0001f;
        uint8_t valid = (uint8_t)(tune.magic == 0xB7U && tune.version == 1U &&
            BigYaw_TuneValid(angle, speed, effort, tau));
        s_big_yaw_tune_sequence = PC_Comm_Data.big_yaw_tune_sequence;
        s_big_yaw_request_id = tune.request_id;
        s_big_yaw_request_status = BigYaw_TuneRequestStatus(valid,
            PC_Comm_Data.big_yaw_tune_rx_safe, PC_Comm_IsRemoteSafe(),
            GimbalPid_YawOutputsOff(), HAL_GetTick() - PC_Comm_Data.big_yaw_tune_rx_time);
        if (s_big_yaw_request_status == 1U) {
            GimbalYaw_TuneBigAngKp = angle;
            GimbalYaw_TuneBigSpdKp = speed;
            GimbalYaw_TuneBigEffortLimit = effort;
            GimbalYaw_TuneBigSpeedFilterTauS = tau;
        }
    }
    __set_PRIMASK(primask);
}

static int16_t GimbalPid_ScaleI16(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 32767.0f) scaled = 32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    return (int16_t)scaled;
}

static uint16_t GimbalPid_ScaleU16(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 65535.0f) scaled = 65535.0f;
    if (scaled < 0.0f) scaled = 0.0f;
    return (uint16_t)scaled;
}

static void GimbalPid_ApplyPcCommands(void)
{
    if (YawIdentApp_OwnsControl()) {
        s_big_yaw_tune_sequence = PC_Comm_Data.big_yaw_tune_sequence;
        s_pid_tune_sequence = PC_Comm_GetGimbalTuneSequence();
        s_pid_action_sequence = PC_Comm_GetGimbalTuneActionSequence();
        s_pitch_tune_sequence = PC_Comm_GetPitchTuneSequence();
        s_pitch_action_sequence = PC_Comm_GetPitchTuneActionSequence();
        return;
    }
    GimbalPid_ApplyBigYawCommand();
    uint32_t sequence = PC_Comm_GetGimbalTuneSequence();
    uint32_t action_sequence = PC_Comm_GetGimbalTuneActionSequence();
    uint32_t pitch_sequence = PC_Comm_GetPitchTuneSequence();
    uint32_t pitch_action_sequence = PC_Comm_GetPitchTuneActionSequence();

    if (sequence != s_pid_tune_sequence) {
        const PC_Recv_GimbalPidTune_t *tune = PC_Comm_GetGimbalTune();
        s_pid_tune_sequence = sequence;
        GimbalYaw_TuneSmallAngKp = (float)tune->angle_kp_milli * 0.001f;
        GimbalYaw_TuneSmallSpdKp = (float)tune->speed_kp_milli * 0.001f;
        GimbalYaw_TuneSmallSpeedLimitRpm =
            YawLimits_DecodeRateCap(tune->speed_limit_centi_rpm, 0.01f);
        GimbalYaw_TuneSmallSpeedFilterAlpha =
            (float)tune->speed_filter_alpha_1e4 * 0.0001f;
        GimbalYaw_TuneSmallManualMaxStepDeg =
            YawLimits_DecodeRateCap(tune->manual_step_1e4_deg, 0.0001f);
        GimbalYaw_DiagEffortLimit = (float)tune->effort_limit_milli * 0.001f;
        GimbalYaw_TuneResetRequest = 1U;
    }

    if (action_sequence != s_pid_action_sequence) {
        const PC_Recv_GimbalPidAction_t *action = PC_Comm_GetGimbalTuneAction();
        s_pid_action_sequence = action_sequence;
        if ((action->flags & PC_GIMBAL_PID_ACTION_RESET) != 0U) {
            GimbalYaw_TuneResetRequest = 1U;
        }
        if (SMALL_YAW_ANGLE_OPEN_TEST == 0U &&
            (action->flags & PC_GIMBAL_PID_ACTION_STEP) != 0U) {
            GimbalYaw_TuneSmallStepDeg = (float)action->step_centi_deg * 0.01f;
        }
    }

    if (pitch_sequence != s_pitch_tune_sequence) {
        const PC_Recv_PitchPidTune_t *tune = PC_Comm_GetPitchTune();
        s_pitch_tune_sequence = pitch_sequence;
        GimbalPitch_TuneImuKp = (float)tune->imu_kp_milli * 0.001f;
        GimbalPitch_TuneImuKd = (float)tune->imu_kd_milli * 0.001f;
        GimbalPitch_TuneRateLimitRadS =
            (float)tune->rate_limit_milli_rad_s * 0.001f;
        GimbalPitch_TuneGravityEffort =
            (float)tune->gravity_effort_milli * 0.001f;
        GimbalPitch_TuneAngleDeadbandRad =
            (float)tune->angle_deadband_1e5_rad * 0.00001f;
        GimbalPitch_TuneRateDeadbandRadS =
            (float)tune->rate_deadband_milli_rad_s * 0.001f;
        GimbalPitch_ImuEnable =
            (uint8_t)((tune->flags & PC_PITCH_PID_TUNE_ENABLE) != 0U);
        GimbalPitch_TuneResetRequest = 1U;
    }

    if (pitch_action_sequence != s_pitch_action_sequence) {
        const PC_Recv_PitchPidAction_t *action =
            PC_Comm_GetPitchTuneAction();
        s_pitch_action_sequence = pitch_action_sequence;
        if ((action->flags & PC_PITCH_PID_ACTION_RESET) != 0U) {
            GimbalPitch_TuneResetRequest = 1U;
        }
        if ((action->flags & PC_PITCH_PID_ACTION_STEP) != 0U) {
            GimbalPitch_TuneStepRad =
                (float)action->step_milli_rad * 0.001f;
        }
        if ((action->flags & PC_PITCH_PID_ACTION_MIT_GAINS) != 0U) {
            RobotActuators_SetPitchMitGains(
                (float)action->mit_kp_centi * 0.01f,
                (float)action->mit_kd_milli * 0.001f);
        }
    }
}

static uint16_t GimbalPid_RoundConfig(float value, float scale)
{
    if (!isfinite(value)) return 0U;
    return GimbalPid_ScaleU16(value * scale + 0.5f, 1.0f);
}

static void GimbalPid_ReadYawEncoder(const Motor_MotorTypeDef *motor,
                                    uint32_t now, uint16_t *raw,
                                    uint16_t *age, uint8_t *valid)
{
    uint32_t elapsed = now - motor->last_update_time;
    float count = motor->encoder.angle;
    *age = (uint16_t)(elapsed > 65535U ? 65535U : elapsed);
    *valid = (uint8_t)(motor->init != 0U && count >= 0.0f && count < 8192.0f);
    *raw = *valid ? (uint16_t)count : 0U;
}

static void GimbalPid_SendYawEncoders(void)
{
    PC_Send_YawEncoders_t data = {0};
    GimbalYaw_GimbalYawTypeDef *yaw = GimbalYaw_GetGimbalYawPtr();
    uint16_t small_raw, big_raw, small_age, big_age;
    uint8_t small_valid, big_valid;
    uint32_t primask = __get_PRIMASK();
    uint32_t now;

    /* Snapshot CAN feedback and its timestamp together, including in SAFE. */
    __disable_irq();
    now = HAL_GetTick();
    GimbalPid_ReadYawEncoder(&Motor_Small_YawMotor, now,
                            &small_raw, &small_age, &small_valid);
    GimbalPid_ReadYawEncoder(&Motor_Big_YawMotor, now,
                            &big_raw, &big_age, &big_valid);
    data.yaw_output_active = (uint8_t)(
        yaw->output_state != 0U ||
        Motor_Small_YawMotor.output != 0.0f || Motor_Big_YawMotor.output != 0.0f);
    __set_PRIMASK(primask);

    data.small_raw_count = small_raw;
    data.big_raw_count = big_raw;
    data.small_age_ms = small_age;
    data.big_age_ms = big_age;
    data.feedback_valid_mask = (uint8_t)(small_valid | (big_valid << 1));
    data.monitor_only = (uint8_t)!PC_Comm_IsGimbalTuneControlLocked();
    data.protocol_version = 1U;
    if (PC_Comm_SendPacket(PC_CMD_YAW_ENCODERS, &data, sizeof(data)))
        s_yaw_encoders_time = now;
}

static void GimbalPid_UpdateYawPeaks(void)
{
    uint16_t raw, age;
    uint8_t valid;
    float speed;
    uint32_t now, primask;
    if (!PC_Comm_IsGimbalTuneSessionOnline() ||
        PC_Comm_GetActiveTuneAxis() != PC_TUNE_AXIS_SMALL_YAW ||
        PC_Comm_IsGimbalTuneControlLocked()) {
        YawPeaks_Reset(&s_big_yaw_peaks);
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    now = HAL_GetTick();
    GimbalPid_ReadYawEncoder(&Motor_Big_YawMotor, now, &raw, &age, &valid);
    speed = Motor_Big_YawMotor.encoder.speed;
    __set_PRIMASK(primask);
    YawPeaks_Update(&s_big_yaw_peaks, now, GimbalYaw_DiagBigActive,
        (uint8_t)(valid && age <= YAW_LIMIT_FEEDBACK_MAX_AGE_MS), raw,
        GimbalYaw_DiagBigErrorDeg, speed, GimbalYaw_DiagBigEffortCmd,
        GimbalYaw_DiagBigSaturated);
}

static void GimbalPid_SendFullConfig(uint32_t now)
{
    if (s_big_yaw_full_part == 0U) {
        float values[BIG_YAW_FULL_COUNT] = {GimbalYaw_TuneBigAngKp,
            GimbalYaw_TuneBigSpdKp, GimbalYaw_TuneBigEffortLimit,
            GimbalYaw_TuneBigSpeedFilterTauS, GimbalYaw_TuneBigAngKi,
            GimbalYaw_TuneBigAngKd, GimbalYaw_TuneBigSpdKi, GimbalYaw_TuneBigSpdKd};
        uint8_t flags = (uint8_t)(s_big_yaw_request_status |
            (PC_Comm_IsRemoteSafe() << 3) | (GimbalPid_YawOutputsOff() << 4) |
            (GimbalYaw_DiagBigActive << 5) |
            ((Motor_Big_YawMotor.init != 0U && now - Motor_Big_YawMotor.last_update_time <=
              YAW_LIMIT_FEEDBACK_MAX_AGE_MS) << 6) | (GimbalYaw_DiagBigSaturated << 7));
        unsigned i;
        for (i = 0; i < 4U; ++i) {
            s_big_yaw_full_snapshot[i][0] = (uint8_t)s_big_yaw_request_id;
            s_big_yaw_full_snapshot[i][1] = (uint8_t)(s_big_yaw_request_id >> 8);
            s_big_yaw_full_snapshot[i][2] = (uint8_t)i;
            s_big_yaw_full_snapshot[i][3] = flags;
            memcpy(&s_big_yaw_full_snapshot[i][4], &values[i * 2U], 8);
        }
    }
    if (PC_Comm_SendPacket(PC_CMD_BIG_YAW_FULL_CONFIG,
                          s_big_yaw_full_snapshot[s_big_yaw_full_part], 12)) {
        if (s_big_yaw_full_part == 0U) s_big_yaw_full_config_time = now;
        s_big_yaw_full_part = (uint8_t)((s_big_yaw_full_part + 1U) % 4U);
    }
}

static void GimbalPid_SendTelemetry(void)
{
    uint32_t now = HAL_GetTick();
    if (YawIdentApp_IsOnline()) return;
    if (PC_Comm_IsYawCaptureOnline()) return;
    if (!PC_Comm_IsGimbalTuneSessionOnline()) return;
    if ((now - s_pid_telemetry_time) < 10U) return;
    s_pid_telemetry_time = now;

    if (PC_Comm_GetActiveTuneAxis() == PC_TUNE_AXIS_SMALL_YAW &&
        !PC_Comm_IsGimbalTuneControlLocked() &&
        (now - s_big_yaw_config_time) >= 100U) {
        PC_Send_BigYawConfig_t data;
        data.angle_kp_milli = GimbalPid_RoundConfig(GimbalYaw_TuneBigAngKp, 1000.0f);
        data.speed_kp_milli = GimbalPid_RoundConfig(GimbalYaw_TuneBigSpdKp, 1000.0f);
        data.effort_milli = GimbalPid_RoundConfig(GimbalYaw_TuneBigEffortLimit, 1000.0f);
        data.filter_tau_1e4_s = GimbalPid_RoundConfig(GimbalYaw_TuneBigSpeedFilterTauS, 10000.0f);
        data.request_id = s_big_yaw_request_id;
        data.status = s_big_yaw_request_status;
        /* Bits 5/6 advertise full effort / complete float32 PID interfaces. */
        data.flags = (uint8_t)(0x80U | 0x20U | 0x40U | PC_Comm_IsRemoteSafe() |
            (GimbalPid_YawOutputsOff() << 1) | (GimbalYaw_DiagBigActive << 2) |
            (GimbalYaw_DiagBigSaturated << 3) |
            ((Motor_Big_YawMotor.init != 0U &&
              now - Motor_Big_YawMotor.last_update_time <= YAW_LIMIT_FEEDBACK_MAX_AGE_MS) << 4));
        if (PC_Comm_SendPacket(PC_CMD_BIG_YAW_CONFIG, &data, sizeof(data)))
            s_big_yaw_config_time = now;
        return;
    }

    if (PC_Comm_GetActiveTuneAxis() == PC_TUNE_AXIS_SMALL_YAW &&
        !PC_Comm_IsGimbalTuneControlLocked() &&
        (s_big_yaw_full_part != 0U || now - s_big_yaw_full_config_time >= 200U)) {
        GimbalPid_SendFullConfig(now);
        return;
    }

    if (PC_Comm_GetActiveTuneAxis() == PC_TUNE_AXIS_SMALL_YAW &&
        !PC_Comm_IsGimbalTuneControlLocked() &&
        (now - s_yaw_encoders_time) >= 50U) {
        GimbalPid_SendYawEncoders();
        return;
    }

    if (PC_Comm_GetActiveTuneAxis() == PC_TUNE_AXIS_SMALL_YAW &&
        !PC_Comm_IsGimbalTuneControlLocked() &&
        (now - s_yaw_limits_time) >= 100U) {
        PC_Send_YawLimits_t data;
        data.joint_centi_deg = GimbalPid_ScaleI16(GimbalYaw_DiagSmallJointDeg, 100.0f);
        data.minimum_centi_deg = GimbalPid_ScaleI16(YAW_LIMIT_MIN_DEG, 100.0f);
        data.maximum_centi_deg = GimbalPid_ScaleI16(YAW_LIMIT_MAX_DEG, 100.0f);
        data.speed_ref_deci_rpm = GimbalPid_ScaleI16(GimbalYaw_DiagSmallSpeedRefRpm, 10.0f);
        data.limits_valid = GimbalYaw_DiagSmallLimitsValid;
        data.limit_status = GimbalYaw_GetGimbalYawPtr()->yaw_ref_limit_status;
        data.big_yaw_enabled = GimbalYaw_DiagBigEnable;
        data.protocol_version = 1U;
        if (PC_Comm_SendPacket(PC_CMD_YAW_LIMITS, &data, sizeof(data)))
            s_yaw_limits_time = now;
        return;
    }

    if (PC_Comm_GetActiveTuneAxis() == PC_TUNE_AXIS_SMALL_YAW &&
        !PC_Comm_IsGimbalTuneControlLocked() &&
        (now - s_yaw_coordinator_time) >= 100U) {
        PC_Send_YawCoordinator_t data;
        data.big_error_centi_deg = GimbalPid_ScaleI16(GimbalYaw_DiagBigErrorDeg, 100.0f);
        data.big_speed_ref_centi_rpm = GimbalPid_ScaleI16(GimbalYaw_DiagBigSpeedRefRpm, 100.0f);
        data.big_speed_fdb_centi_rpm = GimbalPid_ScaleI16(GimbalYaw_DiagBigSpeedRpm, 100.0f);
        data.big_effort_milli = GimbalPid_ScaleI16(GimbalYaw_DiagBigEffortCmd, 1000.0f);
        data.mode = GimbalYaw_DiagBigMode;
        data.active = GimbalYaw_DiagBigActive;
        data.relief_negative = (uint8_t)(GimbalYaw_BigReliefDirection < 0.0f);
        data.protocol_version = 1U;
        if (PC_Comm_SendPacket(PC_CMD_YAW_COORDINATOR, &data, sizeof(data)))
            s_yaw_coordinator_time = now;
        return;
    }

    if (PC_Comm_GetActiveTuneAxis() == PC_TUNE_AXIS_SMALL_YAW &&
        !PC_Comm_IsGimbalTuneControlLocked() &&
        (now - s_big_yaw_peaks_time) >= 100U) {
        uint16_t data[6];
        YawPeaks_Pack(&s_big_yaw_peaks, now, data);
        if (PC_Comm_SendPacket(PC_CMD_BIG_YAW_PEAKS, data, sizeof(data))) {
            s_big_yaw_peaks_time = now;
            YawPeaks_NextWindow(&s_big_yaw_peaks, now);
        }
        return;
    }

    if (PC_Comm_GetActiveTuneAxis() == PC_TUNE_AXIS_PITCH) {
        GimbalPitch_GimbalPitchTypeDef *pitch =
            GimbalPitch_GetGimbalPitchPtr();
        if ((now - s_pitch_limits_time) >= 100U) {
            PC_Send_PitchPidLimits_t limits;
            limits.motor_min_mrad = GimbalPid_ScaleI16(pitch->motor_min_ref, 1000.0f);
            limits.motor_max_mrad = GimbalPid_ScaleI16(pitch->motor_max_ref, 1000.0f);
            limits.imu_min_mrad = GimbalPid_ScaleI16(pitch->imu_min_ref, 1000.0f);
            limits.imu_max_mrad = GimbalPid_ScaleI16(pitch->imu_max_ref, 1000.0f);
            limits.limits_initialized = pitch->limits_initialized;
            limits.dm_state = (uint8_t)motor[Motor1].para.state;
            limits.monitor_only = (uint8_t)!PC_Comm_IsGimbalTuneControlLocked();
            limits.protocol_version = 1U;
            if (PC_Comm_SendPacket(PC_CMD_PITCH_PID_LIMITS, &limits, sizeof(limits)))
                s_pitch_limits_time = now;
            return;
        }
        if (s_pid_telemetry_phase == 0U) {
            PC_Send_PitchPidData_t data;
            data.ref_milli_rad = GimbalPid_ScaleI16(
                pitch->pitch_ref_smooth, 1000.0f);
            data.fdb_milli_rad = GimbalPid_ScaleI16(
                pitch->imu_position_fdb, 1000.0f);
            data.error_milli_rad = GimbalPid_ScaleI16(
                pitch->imu_error, 1000.0f);
            data.motor_ref_milli_rad = GimbalPid_ScaleI16(
                pitch->motor_ref, 1000.0f);
            data.motor_fdb_milli_rad = GimbalPid_ScaleI16(
                GimbalPitch_DiagMotorPosition, 1000.0f);
            data.motor_rate_ref_milli_rad_s = GimbalPid_ScaleI16(
                pitch->motor_rate_ref, 1000.0f);
            if (PC_Comm_SendPacket(PC_CMD_PITCH_PID_DATA,
                                   &data, sizeof(data)) != 0U) {
                s_pid_telemetry_phase = 1U;
            }
        } else {
            PC_Send_PitchPidAux_t data = {0};
            data.imu_rate_milli_rad_s = GimbalPid_ScaleI16(
                pitch->imu_speed_fdb, 1000.0f);
            data.gravity_effort_milli = GimbalPid_ScaleI16(
                GimbalPitch_DiagGravityEffort, 1000.0f);
            data.motor_speed_milli_rad_s = GimbalPid_ScaleI16(
                GimbalPitch_DiagMotorSpeed, 1000.0f);
            data.motor_effort_milli = GimbalPid_ScaleI16(
                GimbalPitch_DiagMotorEffort, 1000.0f);
            data.motor_online = GimbalPitch_DiagMotorOnline;
            data.axis_enabled = (uint8_t)(GimbalPitch_ImuEnable != 0U &&
                                          pitch->output_state == 1U &&
                                          pitch->control_state == 1U &&
                                          INS_IsReady() &&
                                          GimbalPitch_DiagMotorOnline != 0U &&
                                          pitch->limits_initialized != 0U);
            data.applied_step_milli_rad = GimbalPid_ScaleI16(
                GimbalPitch_DiagAppliedStepRad, 1000.0f);
            if (PC_Comm_SendPacket(PC_CMD_PITCH_PID_AUX,
                                   &data, sizeof(data)) != 0U) {
                s_pid_telemetry_phase = 0U;
            }
        }
    } else if (s_pid_telemetry_phase == 0U) {
        PC_Send_GimbalPidData_t data;
        data.ref_centi_deg = GimbalPid_ScaleU16(GimbalYaw_DiagSmallRefDeg, 100.0f);
        data.fdb_centi_deg = GimbalPid_ScaleU16(GimbalYaw_DiagSmallFdbDeg, 100.0f);
        data.error_centi_deg = GimbalPid_ScaleI16(GimbalYaw_DiagSmallErrorDeg, 100.0f);
        data.speed_ref_deci_rpm = GimbalPid_ScaleI16(GimbalYaw_DiagSmallSpeedRefRpm, 10.0f);
        data.speed_deci_rpm = GimbalPid_ScaleI16(GimbalYaw_DiagSmallSpeedRpm, 10.0f);
        data.effort_command = GimbalPid_ScaleI16(GimbalYaw_DiagSmallEffortCmd, 1000.0f);
        if (PC_Comm_SendPacket(PC_CMD_GIMBAL_PID_DATA, &data, sizeof(data)) != 0U) {
            s_pid_telemetry_phase = 1U;
        }
    } else {
        PC_Send_GimbalPidAux_t data = {0};
        data.raw_speed_deci_rpm = GimbalPid_ScaleI16(GimbalYaw_DiagSmallRawSpeedRpm, 10.0f);
        data.current = GimbalPid_ScaleI16(GimbalYaw_DiagSmallCurrent, 1.0f);
        data.motor_online = Motor_Small_YawMotor.is_online;
        data.axis_enabled = GimbalYaw_DiagSmallEnable;
        if (PC_Comm_SendPacket(PC_CMD_GIMBAL_PID_AUX, &data, sizeof(data)) != 0U) {
            s_pid_telemetry_phase = 0U;
        }
    }
}

void Gimbal_Task(void const *argument)
{
    uint32_t cycle = 0U;
    (void)argument;

    MyUART_Init();
    (void)DWT_GetDeltaT(&cycle);
    for (;;) {
        s_gimbal_dt_s = DWT_GetDeltaT(&cycle);
        AttitudeLink_Service();
        GimbalPid_ApplyPcCommands();
        YawIdentApp_Pre();
        Tidy_send_vision(&visionDataSend);
        SendVisionData(&visionDataSend);
        Shooter_UpdataControlData();
        GimbalPitch_Control();
        GimbalPitch_Output();
        GimbalYaw_Control();
        GimbalPid_UpdateYawPeaks();
        GimbalPid_SendTelemetry();
        Shooter_FeederControl();
        GimbalYaw_Output();
        YawIdentApp_Post();
        GimbalCapture_Service();
        AttitudeLink_Service();
        osDelay(1U);
    }
}
