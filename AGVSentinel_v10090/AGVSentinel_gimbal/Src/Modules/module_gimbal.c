/*
 *  Project      : Polaris
 * 
 *  file         : cha_gimbal_ctrl.c
 *  Description  : This file contains Gimbal Pitch control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 16:27:28
 */


#include "cmsis_os.h"
#include "sys_const.h"
#include "module_gimbal.h"
#include "app_yaw_identification.h"
#include "module_gimbal_axis.h"
#include "module_pitch_limits.h"
#include "module_yaw_limits.h"
#include "module_yaw_coordinator.h"
#include "module_yaw_mpc.h"
#include "module_big_yaw_tune.h"
#include "module_big_yaw_manual.h"
#include "periph_remote.h"
#include "periph_pc_comm.h"
#include "periph_DMmotor.h"
#include "periph_motor.h"
#include "sys_robot_actuators.h"
#include "protocol_common.h"
#include "app_ins.h"

#define GIMBAL_RAD_TO_RPM (9.549296586f)
#define GIMBAL_CONTROL_DT_S (0.001f)
#define GIMBAL_PITCH_MOTOR_REF_LEAD_MAX_RAD (0.08f)
#define GIMBAL_PITCH_MOTOR_REV_RAD (6.283185307f)
#define GIMBAL_PITCH_LIMIT_MARGIN_RAD (0.005f)

static float GimbalYaw_Normalize360(float ang) {
    while (ang < 0.0f) ang += 360.0f;
    while (ang >= 360.0f) ang -= 360.0f;
    return ang;
}

static float GimbalYaw_Wrap180(float ang) {
    while (ang > 180.0f) ang -= 360.0f;
    while (ang < -180.0f) ang += 360.0f;
    return ang;
}

static float Gimbal_Clamp(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float Gimbal_ApplyDeadband(float value, float deadband) {
    if (value > deadband) return value - deadband;
    if (value < -deadband) return value + deadband;
    return 0.0f;
}

static uint8_t GimbalImu_IsReady(void) {
    return INS_IsReady();
}

static void GimbalPitch_InitializeMechanicalLimits(
    GimbalPitch_GimbalPitchTypeDef *pitch,
    float motor_position,
    float imu_position)
{
    if (pitch == NULL) return;
    pitch->limits_initialized = PitchLimits_Align(
        motor_position, Const_PITCH_DMAXANGLE, Const_PITCH_UMAXANGLE,
        GIMBAL_PITCH_LIMIT_MARGIN_RAD, motor[Motor1].tmp.PMAX,
        &pitch->motor_min_ref, &pitch->motor_max_ref);
    if (pitch->limits_initialized == 0U) return;
    pitch->motor_zero_ref = 0.5f * (pitch->motor_min_ref + pitch->motor_max_ref);
    pitch->imu_zero_ref = imu_position;
    pitch->imu_min_ref = imu_position;
    pitch->imu_max_ref = imu_position;
}

GimbalPitch_GimbalPitchTypeDef GimbalPitch_gimbalPitchControlData;
GimbalYaw_GimbalYawTypeDef GimbalYaw_gimbalYawControlData;
static GimbalAxis_TypeDef s_pitch_axis;
static GimbalAxis_TypeDef s_big_yaw_axis;
static GimbalAxis_TypeDef s_small_yaw_axis;

volatile float GimbalYaw_DiagBigEffortCmd;
volatile float GimbalYaw_DiagBigErrorDeg;
volatile float GimbalYaw_DiagBigSpeedRefRpm;
volatile float GimbalYaw_DiagBigSpeedRpm;
volatile float GimbalYaw_DiagBigCurrent;
volatile float GimbalYaw_DiagBigTemperatureC;
volatile uint8_t GimbalYaw_DiagBigSaturated;
volatile uint8_t GimbalYaw_DiagBigMode;
volatile uint8_t GimbalYaw_DiagBigActive;
volatile uint8_t GimbalYaw_DiagBigStopReason;
volatile float GimbalYaw_DiagSmallEffortCmd;
volatile float GimbalYaw_DiagSmallRefDeg;
volatile float GimbalYaw_DiagSmallFdbDeg;
volatile float GimbalYaw_DiagSmallRawSpeedRpm;
volatile float GimbalYaw_DiagSmallSpeedRpm;
volatile float GimbalYaw_DiagSmallErrorDeg;
volatile float GimbalYaw_DiagSmallSpeedRefRpm;
volatile float GimbalYaw_DiagSmallCurrent;
volatile float GimbalYaw_DiagSmallJointDeg;
volatile uint8_t GimbalYaw_DiagSmallLimitsValid;
volatile uint8_t GimbalYaw_DiagMpcRequested;
volatile uint8_t GimbalYaw_DiagMpcActive;
volatile uint8_t GimbalYaw_DiagMpcUpdated;
volatile uint8_t GimbalYaw_DiagMpcReason;
volatile float GimbalYaw_DiagMpcBigRefDps;
volatile float GimbalYaw_DiagMpcSmallRefDps;
volatile float GimbalYaw_DiagMpcBigUnconstrainedDps;
volatile float GimbalYaw_DiagMpcSmallUnconstrainedDps;

volatile float GimbalYaw_TuneBigAngKp;
volatile float GimbalYaw_TuneBigSpdKp;
volatile float GimbalYaw_TuneBigAngKi;
volatile float GimbalYaw_TuneBigAngKd;
volatile float GimbalYaw_TuneBigSpdKi;
volatile float GimbalYaw_TuneBigSpdKd;
volatile float GimbalYaw_TuneBigEffortLimit;
volatile float GimbalYaw_TuneBigSpeedFilterTauS;

volatile float GimbalYaw_TuneSmallAngKp;
volatile float GimbalYaw_TuneSmallAngKi;
volatile float GimbalYaw_TuneSmallAngKd;
volatile float GimbalYaw_TuneSmallSpdKp;
volatile float GimbalYaw_TuneSmallSpdKi;
volatile float GimbalYaw_TuneSmallSpdKd;
volatile float GimbalYaw_TuneSmallSpeedLimitRpm;
volatile float GimbalYaw_TuneSmallSpeedFilterAlpha;
volatile float GimbalYaw_TuneSmallManualMaxStepDeg;
volatile float GimbalYaw_TuneSmallStepDeg;
volatile uint8_t GimbalYaw_TuneResetRequest;

volatile float GimbalPitch_TuneImuKp;
volatile float GimbalPitch_TuneImuKd;
volatile float GimbalPitch_TuneRateLimitRadS;
volatile float GimbalPitch_TuneGravityEffort;
volatile float GimbalPitch_TuneAngleDeadbandRad;
volatile float GimbalPitch_TuneRateDeadbandRadS;
volatile float GimbalPitch_TuneStepRad;
volatile float GimbalPitch_DiagAppliedStepRad;
volatile uint8_t GimbalPitch_TuneResetRequest;
volatile float GimbalPitch_DiagGravityEffort;
volatile float GimbalPitch_DiagMotorPosition;
volatile float GimbalPitch_DiagMotorSpeed;
volatile float GimbalPitch_DiagMotorEffort;
volatile uint8_t GimbalPitch_DiagMotorOnline;

static uint8_t s_big_yaw_was_enabled;
static uint8_t s_big_yaw_was_direct_speed;
static uint8_t s_small_yaw_was_enabled;
static uint8_t s_pitch_was_enabled;
static float s_small_yaw_speed_filtered;
static SmallYawRate_Command s_small_yaw_rate_command;
static YawCoordinator_State s_yaw_coordinator;
static uint32_t s_big_yaw_control_time;
static BigYawManual_Command s_big_yaw_manual_command;
static BigYawManual_State s_big_yaw_manual;
typedef struct {
    uint8_t enabled;
    float target_rate_dps;
    uint32_t tick_ms;
} GimbalYaw_MpcRequest;
static GimbalYaw_MpcRequest s_yaw_mpc_request;
static YawMpc_State s_yaw_mpc;
volatile uint8_t GimbalYaw_DiagBigManualFault;

void GimbalYaw_SetMpcMode(uint8_t enabled, float target_rate_dps)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_yaw_mpc_request.enabled = enabled;
    s_yaw_mpc_request.target_rate_dps = target_rate_dps;
    s_yaw_mpc_request.tick_ms = HAL_GetTick();
    __set_PRIMASK(primask);
}

static GimbalYaw_MpcRequest GimbalYaw_ReadMpcRequest(void)
{
    uint32_t primask = __get_PRIMASK();
    GimbalYaw_MpcRequest request;
    __disable_irq();
    request = s_yaw_mpc_request;
    __set_PRIMASK(primask);
    return request;
}

void GimbalYaw_SetBigYawManualRate(uint8_t enabled, float rate_dps)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    BigYawManual_Publish(&s_big_yaw_manual_command, enabled, rate_dps, HAL_GetTick());
    __set_PRIMASK(primask);
}

static BigYawManual_Command GimbalYaw_ReadBigManualCommand(void)
{
    uint32_t primask = __get_PRIMASK();
    BigYawManual_Command command;
    __disable_irq();
    command = s_big_yaw_manual_command;
    __set_PRIMASK(primask);
    return command;
}

static uint8_t GimbalYaw_BigManualPermitted(uint32_t now)
{
    uint32_t primask = __get_PRIMASK();
    Remote_RemoteDataTypeDef remote;
    uint8_t fresh;
    __disable_irq();
    remote = *Remote_GetRemoteDataPtr();
    __set_PRIMASK(primask);
    fresh = (uint8_t)(remote.state == Remote_STATE_CONNECTED &&
        now - remote.last_update_time <= BIG_YAW_MANUAL_TIMEOUT_MS);
    if (fresh && remote.remote.s[1] == Remote_SWITCH_UP) {
        s_big_yaw_manual.fault = 0U;
        s_big_yaw_manual.active = 0U;
    }
    GimbalYaw_DiagBigManualFault = s_big_yaw_manual.fault;
    return (uint8_t)(fresh && remote.remote.s[1] == Remote_SWITCH_DOWN &&
        remote.remote.s[0] == Remote_SWITCH_DOWN &&
        !PC_Comm_IsGimbalTuneControlLocked() && !s_big_yaw_manual.fault);
}

static float GimbalYaw_ClampEffort(float effort) {
    float limit = GimbalYaw_DiagEffortLimit;
    if (limit < 0.0f) limit = -limit;
    if (effort > limit) return limit;
    if (effort < -limit) return -limit;
    return effort;
}

static void GimbalYaw_ClearBigController(GimbalYaw_GimbalYawTypeDef *yaw) {
    BigYaw_ResetPid(&yaw->angPID);
    BigYaw_ResetPid(&yaw->spdPID);
}

static void GimbalYaw_StopBig(GimbalYaw_GimbalYawTypeDef *yaw)
{
    s_big_yaw_was_enabled = 0U;
    s_big_yaw_was_direct_speed = 0U;
    s_big_yaw_manual.active = 0U;
    YawCoordinator_Reset(&s_yaw_coordinator);
    GimbalYaw_ClearBigController(yaw);
    GimbalYaw_DiagBigErrorDeg = 0.0f;
    GimbalYaw_DiagBigEffortCmd = 0.0f;
    GimbalYaw_DiagBigSpeedRefRpm = 0.0f;
    GimbalYaw_DiagBigSpeedRpm = 0.0f;
    GimbalYaw_DiagBigSaturated = 0U;
    GimbalYaw_DiagBigMode = YAW_COORD_DISABLED;
    GimbalYaw_DiagBigActive = 0U;
    GimbalAxis_SetEffort(&s_big_yaw_axis, 0.0f);
}

static void GimbalYaw_ControlBig(GimbalYaw_GimbalYawTypeDef *yaw,
    const Actuator_FeedbackTypeDef *feedback, float small_deg,
    float small_rate_rpm)
{
    uint32_t now = HAL_GetTick();
    float dt = s_big_yaw_was_enabled ?
        (float)(now - s_big_yaw_control_time) * 0.001f : GIMBAL_CONTROL_DT_S;
    float error, speed, effort;
    float effort_limit = GimbalYaw_TuneBigEffortLimit;
    float filter_tau_s = GimbalYaw_TuneBigSpeedFilterTauS;
    float values[BIG_YAW_FULL_COUNT] = {GimbalYaw_TuneBigAngKp,
        GimbalYaw_TuneBigSpdKp, effort_limit, filter_tau_s, GimbalYaw_TuneBigAngKi,
        GimbalYaw_TuneBigAngKd, GimbalYaw_TuneBigSpdKi, GimbalYaw_TuneBigSpdKd};
    float previous_angle_sum, previous_speed_sum;
    uint8_t tune_valid = BigYaw_ApplyFullTune(&yaw->angPIDParam, &yaw->spdPIDParam, values);
    float relief_direction = GimbalYaw_BigReliefDirection;
    uint8_t identifying = YawIdentApp_OwnsControl();
    uint8_t speed_identifying = YawIdentApp_UsesSpeedReference();
    uint8_t mpc_active = (uint8_t)(!identifying && GimbalYaw_DiagMpcActive != 0U);
    uint8_t direct_speed = (uint8_t)(speed_identifying || mpc_active);
    BigYawManual_Command manual_command = GimbalYaw_ReadBigManualCommand();
    uint8_t manual = (uint8_t)(!identifying && GimbalYaw_BigManualPermitted(now) &&
        BigYawManual_Fresh(&manual_command, now));
    uint8_t manual_entry = (uint8_t)(manual && (!s_big_yaw_manual.active ||
        s_big_yaw_manual.generation != manual_command.generation));
    uint8_t allow = (uint8_t)(identifying ? YawIdentApp_AllowYaw() :
        (manual || mpc_active ||
         (BIG_YAW_OUTPUT_INHIBIT_TEST == 0U && GimbalYaw_DiagBigEnable != 0U)));
    uint8_t stop_reason = GIMBAL_BIG_STOP_NONE;
    if (!allow) stop_reason = GIMBAL_BIG_STOP_ALLOW;
    else if (!tune_valid || effort_limit <= 0.0f) stop_reason = GIMBAL_BIG_STOP_TUNE;
    else if (!isfinite(GimbalYaw_DiagEffortLimit) || GimbalYaw_DiagEffortLimit <= 0.0f)
        stop_reason = GIMBAL_BIG_STOP_EFFORT_LIMIT;
    else if (!feedback->online || !isfinite(feedback->position) || !isfinite(feedback->velocity) ||
             !Motor_IsMotorFeedbackFresh(&Motor_Big_YawMotor,
                                          YAW_LIMIT_FEEDBACK_MAX_AGE_MS))
        stop_reason = GIMBAL_BIG_STOP_FEEDBACK;
    else if (!s_small_yaw_was_enabled &&
             !(BIG_YAW_PASSIVE_SMALL_TEST != 0U && GimbalImu_IsReady() != 0U))
        stop_reason = GIMBAL_BIG_STOP_SMALL_INTERLOCK;
    else if (!GimbalYaw_DiagSmallLimitsValid) stop_reason = GIMBAL_BIG_STOP_SMALL_LIMITS;
    else if (GimbalYaw_DiagBigDirection != 1.0f) stop_reason = GIMBAL_BIG_STOP_DIRECTION;
    s_big_yaw_control_time = now;
    if (stop_reason != GIMBAL_BIG_STOP_NONE) {
        /* After an internal stop, the identification supervisor withdraws
         * allow on the next cycle.  Preserve the specific branch until a
         * successful control cycle clears it instead of masking it as ALLOW. */
        if (stop_reason != GIMBAL_BIG_STOP_ALLOW ||
            GimbalYaw_DiagBigStopReason == GIMBAL_BIG_STOP_NONE)
            GimbalYaw_DiagBigStopReason = stop_reason;
        GimbalYaw_StopBig(yaw);
        return;
    }
    if (dt == 0.0f) {
        GimbalYaw_DiagBigStopReason = GIMBAL_BIG_STOP_NONE;
        return;
    }
    if (!s_big_yaw_was_enabled || manual_entry ||
        direct_speed != s_big_yaw_was_direct_speed ||
        s_yaw_coordinator.relief_direction != relief_direction) {
        GimbalYaw_ClearBigController(yaw);
        GimbalYaw_DiagBigSpeedRefRpm = 0.0f;
        GimbalYaw_DiagBigSpeedRpm = feedback->velocity;
    }
    if (identifying) {
        s_big_yaw_manual.active = 0U;
        s_yaw_coordinator.reference_deg = YawCoordinator_Wrap(speed_identifying ?
            feedback->position : YawIdentApp_BigReference());
        s_yaw_coordinator.feedforward_dps = 0.0f;
        s_yaw_coordinator.mode = YAW_COORD_HOLD;
        s_yaw_coordinator.relief_direction = relief_direction;
    } else if (manual) {
        if (!BigYawManual_Step(&s_big_yaw_manual, &manual_command, now, 1U,
            feedback->position, feedback->velocity, dt)) {
            GimbalYaw_DiagBigManualFault = s_big_yaw_manual.fault;
            GimbalYaw_DiagBigStopReason = GIMBAL_BIG_STOP_COORDINATOR;
            GimbalYaw_StopBig(yaw);
            return;
        }
        s_yaw_coordinator.reference_deg = s_big_yaw_manual.reference_deg;
        s_yaw_coordinator.feedforward_dps = 0.0f;
        s_yaw_coordinator.mode = YAW_COORD_HOLD;
        s_yaw_coordinator.relief_direction = relief_direction;
    } else if (mpc_active) {
        s_big_yaw_manual.active = 0U;
        s_yaw_coordinator.reference_deg = YawCoordinator_Wrap(feedback->position);
        s_yaw_coordinator.feedforward_dps = 0.0f;
        s_yaw_coordinator.mode = YAW_COORD_HOLD;
        s_yaw_coordinator.relief_direction = relief_direction;
    } else if (!YawCoordinator_Step(&s_yaw_coordinator, small_deg,
        small_rate_rpm * 6.0f, feedback->position, relief_direction, dt, 1U)) {
        GimbalYaw_DiagBigStopReason = GIMBAL_BIG_STOP_COORDINATOR;
        GimbalYaw_StopBig(yaw);
        return;
    }
    s_big_yaw_was_enabled = 1U;
    s_big_yaw_was_direct_speed = direct_speed;
    GimbalYaw_DiagBigActive = 1U;
    GimbalYaw_DiagBigMode = (uint8_t)s_yaw_coordinator.mode;
    GimbalYaw_DiagBigSpeedRpm += dt / (filter_tau_s + dt) *
        (feedback->velocity - GimbalYaw_DiagBigSpeedRpm);
    error = direct_speed ? 0.0f :
        remainderf(s_yaw_coordinator.reference_deg - feedback->position, 360.0f);
    GimbalYaw_DiagBigErrorDeg = error;
    /* Both policies use full angle error; the coordinator owns target generation. */
    previous_angle_sum = yaw->angPID.sum;
    previous_speed_sum = yaw->spdPID.sum;
    if (direct_speed) {
        BigYaw_ResetPid(&yaw->angPID);
        speed = speed_identifying ? YawIdentApp_BigSpeedReferenceRpm() :
            GimbalYaw_DiagMpcBigRefDps / 6.0f;
    } else {
        PID_SetPIDRef(&yaw->angPID, error);
        PID_SetPIDFdb(&yaw->angPID, 0.0f);
        if (!BigYaw_CalcPid(&yaw->angPID, &yaw->angPIDParam)) {
            GimbalYaw_DiagBigStopReason = GIMBAL_BIG_STOP_ANGLE_PID;
            GimbalYaw_StopBig(yaw);
            return;
        }
        speed = PID_GetPIDOutput(&yaw->angPID) + s_yaw_coordinator.feedforward_dps / 6.0f;
    }
    if (!isfinite(speed) || !BigYaw_PidFinite(&yaw->angPID)) {
        GimbalYaw_DiagBigStopReason = GIMBAL_BIG_STOP_SPEED_REFERENCE;
        GimbalYaw_StopBig(yaw);
        return;
    }
    GimbalYaw_DiagBigSpeedRefRpm = speed;
    PID_SetPIDRef(&yaw->spdPID, speed);
    PID_SetPIDFdb(&yaw->spdPID, GimbalYaw_DiagBigSpeedRpm);
    if (!BigYaw_CalcPid(&yaw->spdPID, &yaw->spdPIDParam)) {
        GimbalYaw_DiagBigStopReason = GIMBAL_BIG_STOP_SPEED_PID;
        GimbalYaw_StopBig(yaw);
        return;
    }
    effort = PID_GetPIDOutput(&yaw->spdPID);
    if (!isfinite(effort) || !BigYaw_PidFinite(&yaw->spdPID)) {
        GimbalYaw_DiagBigStopReason = GIMBAL_BIG_STOP_EFFORT_OUTPUT;
        GimbalYaw_StopBig(yaw);
        return;
    }
    GimbalYaw_DiagBigSaturated = (uint8_t)(yaw->spdPID.err_lim != 0.0f);
    if (!direct_speed)
        BigYaw_Unwind(&yaw->angPID, previous_angle_sum, error, yaw->spdPID.err_lim);
    BigYaw_Unwind(&yaw->spdPID, previous_speed_sum, yaw->spdPID.err[0], yaw->spdPID.err_lim);
    /* Independent effort budget; a small-yaw tuning cap must not weaken big yaw. */
    GimbalYaw_DiagBigEffortCmd = YawLimits_Clamp(effort, -effort_limit, effort_limit);
    if (identifying || manual || mpc_active) GimbalAxis_SetEnabled(&s_big_yaw_axis, 1U);
    GimbalAxis_SetEffort(&s_big_yaw_axis, GimbalYaw_DiagBigEffortCmd);
    GimbalYaw_DiagBigStopReason = GIMBAL_BIG_STOP_NONE;
}

static void GimbalYaw_ClearSmallController(GimbalYaw_GimbalYawTypeDef *yaw) {
    PID_ClearPID(&yaw->GimbalSmallYaw_angPID);
    PID_ClearPID(&yaw->GimbalSmallYaw_spdPID);
}

static float GimbalYaw_ClampTuneValue(volatile float *value,
                                      float min_value,
                                      float max_value,
                                      float fallback) {
    float result = *value;

    if (result != result) result = fallback;
    if (result < min_value) result = min_value;
    if (result > max_value) result = max_value;
    *value = result;
    return result;
}

static void GimbalYaw_ApplySmallTune(GimbalYaw_GimbalYawTypeDef *yaw) {
    PID_PIDParamTypeDef *ang = &yaw->GimbalSmallYaw_angPIDParam;
    PID_PIDParamTypeDef *spd = &yaw->GimbalSmallYaw_spdPIDParam;
    float speed_limit;
    float effort_limit;

    ang->kp = GimbalYaw_ClampTuneValue(&GimbalYaw_TuneSmallAngKp,
                                       0.0f, 5.0f, 2.60f);
    ang->ki = GimbalYaw_ClampTuneValue(&GimbalYaw_TuneSmallAngKi,
                                       0.0f, 1.0f, 0.0f);
    ang->kd = GimbalYaw_ClampTuneValue(&GimbalYaw_TuneSmallAngKd,
                                       0.0f, 10.0f, 0.0f);
    spd->kp = GimbalYaw_ClampTuneValue(&GimbalYaw_TuneSmallSpdKp,
                                       0.0f, 50.0f, 0.60f);
    spd->ki = GimbalYaw_ClampTuneValue(&GimbalYaw_TuneSmallSpdKi,
                                       0.0f, 5.0f, 0.0f);
    spd->kd = GimbalYaw_ClampTuneValue(&GimbalYaw_TuneSmallSpdKd,
                                       0.0f, 20.0f, 0.0f);

    if (GimbalYaw_TuneSmallSpeedLimitRpm != YAW_RATE_UNLIMITED)
        (void)GimbalYaw_ClampTuneValue(&GimbalYaw_TuneSmallSpeedLimitRpm,
                                     0.0f, 30.0f, 24.0f);
    speed_limit = YawLimits_RateCap(GimbalYaw_TuneSmallSpeedLimitRpm);
    (void)GimbalYaw_ClampTuneValue(&GimbalYaw_TuneSmallSpeedFilterAlpha,
                                   0.01f, 1.0f, 0.15f);
    if (GimbalYaw_TuneSmallManualMaxStepDeg != YAW_RATE_UNLIMITED)
        (void)GimbalYaw_ClampTuneValue(&GimbalYaw_TuneSmallManualMaxStepDeg,
                                     0.0f, 0.5f, 0.30f);
    effort_limit = GimbalYaw_ClampTuneValue(&GimbalYaw_DiagEffortLimit,
                                             0.0f, 8.0f, 6.0f);
    ang->output_max = speed_limit;
    spd->output_max = effort_limit;

    if (GimbalYaw_TuneResetRequest != 0U) {
        GimbalYaw_ClearSmallController(yaw);
        GimbalYaw_TuneResetRequest = 0U;
    }
}

/**
  * @brief      Gimbal pitch control initialization
  * @param      NULL
  * @retval     NULL
  */
void GimbalPitch_InitGimbalPitch() {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();

    GimbalAxis_Init(&s_pitch_axis,
                    RobotActuators_GetMotor(ROBOT_MOTOR_GIMBAL_PITCH),
                    RobotActuators_GetGroup(ROBOT_GROUP_GIMBAL_PITCH),
                    0.0f);

    gimbalpitch->control_state = 1;
    gimbalpitch->output_state = 1;
    gimbalpitch->pitch_ref = 0;
    gimbalpitch->pitch_ref_smooth = 0.0f;
    gimbalpitch->filter_alpha = Const_GimbalPitchTargetFilterAlpha;
    gimbalpitch->motor_ref = 0.0f;
    gimbalpitch->imu_position_fdb = 0.0f;
    gimbalpitch->imu_speed_fdb = 0.0f;
    gimbalpitch->imu_error = 0.0f;
    gimbalpitch->motor_rate_ref = 0.0f;
    gimbalpitch->motor_zero_ref = 0.0f;
    gimbalpitch->motor_min_ref = 0.0f;
    gimbalpitch->motor_max_ref = 0.0f;
    gimbalpitch->imu_zero_ref = 0.0f;
    gimbalpitch->imu_min_ref = 0.0f;
    gimbalpitch->imu_max_ref = 0.0f;
    gimbalpitch->limits_initialized = 0U;
    gimbalpitch->pitch_count = 0;
    s_pitch_was_enabled = 0U;
    GimbalPitch_TuneImuKp = Const_GimbalPitchImuKp;
    GimbalPitch_TuneImuKd = Const_GimbalPitchImuKd;
    GimbalPitch_TuneRateLimitRadS = Const_GimbalPitchImuRateMaxRadS;
    GimbalPitch_TuneGravityEffort = 0.30f;
    GimbalPitch_TuneAngleDeadbandRad = Const_GimbalPitchImuAngleDeadbandRad;
    GimbalPitch_TuneRateDeadbandRadS = Const_GimbalPitchImuRateDeadbandRadS;
    GimbalPitch_TuneStepRad = 0.0f;
    GimbalPitch_DiagAppliedStepRad = 0.0f;
    GimbalPitch_TuneResetRequest = 0U;
    GimbalPitch_DiagGravityEffort = 0.0f;
    GimbalPitch_DiagMotorPosition = 0.0f;
    GimbalPitch_DiagMotorSpeed = 0.0f;
    GimbalPitch_DiagMotorEffort = 0.0f;
    GimbalPitch_DiagMotorOnline = 0U;

    PID_InitPIDParam(&gimbalpitch->spdPIDParam, Const_GimbalPitchSpdParam[0][0], Const_GimbalPitchSpdParam[0][1], Const_GimbalPitchSpdParam[0][2], Const_GimbalPitchSpdParam[0][3], 
                    Const_GimbalPitchSpdParam[0][4], Const_GimbalPitchSpdParam[1][0], Const_GimbalPitchSpdParam[1][1], Const_GimbalPitchSpdParam[2][0], Const_GimbalPitchSpdParam[2][1], 
                    Const_GimbalPitchSpdParam[3][0], Const_GimbalPitchSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&gimbalpitch->angPIDParam, Const_GimbalPitchAngParam[0][0], Const_GimbalPitchAngParam[0][1], Const_GimbalPitchAngParam[0][2], Const_GimbalPitchAngParam[0][3], 
                    Const_GimbalPitchAngParam[0][4], Const_GimbalPitchAngParam[1][0], Const_GimbalPitchAngParam[1][1], Const_GimbalPitchAngParam[2][0], Const_GimbalPitchAngParam[2][1], 
                    Const_GimbalPitchAngParam[3][0], Const_GimbalPitchAngParam[3][1], PID_POSITION);                      
}

/**
  * @brief      Gimbal yaw control initialization
  * @param      NULL
  * @retval     NULL
  */
void GimbalYaw_InitGimbalYaw() {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    memset(&s_big_yaw_manual_command, 0, sizeof(s_big_yaw_manual_command));
    memset(&s_big_yaw_manual, 0, sizeof(s_big_yaw_manual));
    memset(&s_yaw_mpc_request, 0, sizeof(s_yaw_mpc_request));
    YawMpc_Reset(&s_yaw_mpc, YAW_MPC_DISABLED);
    GimbalYaw_DiagBigManualFault = 0U;
    GimbalYaw_DiagBigStopReason = GIMBAL_BIG_STOP_NONE;
    GimbalYaw_DiagMpcRequested = 0U;
    GimbalYaw_DiagMpcActive = 0U;
    GimbalYaw_DiagMpcUpdated = 0U;
    GimbalYaw_DiagMpcReason = YAW_MPC_DISABLED;
    GimbalYaw_DiagMpcBigRefDps = 0.0f;
    GimbalYaw_DiagMpcSmallRefDps = 0.0f;
    GimbalYaw_DiagMpcBigUnconstrainedDps = 0.0f;
    GimbalYaw_DiagMpcSmallUnconstrainedDps = 0.0f;

    GimbalAxis_Init(&s_big_yaw_axis,
                    RobotActuators_GetMotor(ROBOT_MOTOR_GIMBAL_BIG_YAW),
                    RobotActuators_GetGroup(ROBOT_GROUP_GIMBAL_BIG_YAW),
                    Const_GimbalBigYawAngleOffset);
    GimbalAxis_SetEnabled(&s_big_yaw_axis, !BIG_YAW_OUTPUT_INHIBIT_TEST);
    GimbalAxis_Init(&s_small_yaw_axis,
                    RobotActuators_GetMotor(ROBOT_MOTOR_GIMBAL_SMALL_YAW),
                    RobotActuators_GetGroup(ROBOT_GROUP_GIMBAL_SMALL_YAW),
                    Const_GimbalSmallYawAngleOffset);

    gimbalyaw->control_state = 1;
    gimbalyaw->output_state = 1;
    gimbalyaw->yaw_ref = 0;
    gimbalyaw->yaw_count = 0;
    GimbalYaw_StopBig(gimbalyaw);
    s_small_yaw_was_enabled = 0U;
    GimbalYaw_DiagBigEffortCmd = 0.0f;
    GimbalYaw_DiagSmallEffortCmd = 0.0f;
    GimbalYaw_DiagSmallRefDeg = 0.0f;
    GimbalYaw_DiagSmallFdbDeg = 0.0f;
    GimbalYaw_DiagSmallRawSpeedRpm = 0.0f;
    GimbalYaw_DiagSmallSpeedRpm = 0.0f;
    GimbalYaw_DiagSmallErrorDeg = 0.0f;
    GimbalYaw_DiagSmallSpeedRefRpm = 0.0f;
    GimbalYaw_DiagSmallCurrent = 0.0f;
    GimbalYaw_TuneBigAngKp = Const_GimbalYawAngParam[0][0];
    GimbalYaw_TuneBigSpdKp = Const_GimbalYawSpdParam[0][0];
    GimbalYaw_TuneBigAngKi = Const_GimbalYawAngParam[0][1];
    GimbalYaw_TuneBigAngKd = Const_GimbalYawAngParam[0][2];
    GimbalYaw_TuneBigSpdKi = Const_GimbalYawSpdParam[0][1];
    GimbalYaw_TuneBigSpdKd = Const_GimbalYawSpdParam[0][2];
    GimbalYaw_TuneBigEffortLimit = Const_GimbalYawSpdParam[0][4];
    GimbalYaw_TuneBigSpeedFilterTauS = BIG_YAW_SPEED_FILTER_TAU_S;
    GimbalYaw_TuneSmallAngKp = Const_GimbalSmallYawAngParam[0][0];
    GimbalYaw_TuneSmallAngKi = Const_GimbalSmallYawAngParam[0][1];
    GimbalYaw_TuneSmallAngKd = Const_GimbalSmallYawAngParam[0][2];
    GimbalYaw_TuneSmallSpdKp = Const_GimbalSmallYawSpdParam[0][0];
    GimbalYaw_TuneSmallSpdKi = Const_GimbalSmallYawSpdParam[0][1];
    GimbalYaw_TuneSmallSpdKd = Const_GimbalSmallYawSpdParam[0][2];
    GimbalYaw_TuneSmallSpeedLimitRpm = YAW_RATE_UNLIMITED;
    GimbalYaw_TuneSmallSpeedFilterAlpha = 0.15f;
    GimbalYaw_TuneSmallManualMaxStepDeg = YAW_RATE_UNLIMITED;
    GimbalYaw_TuneSmallStepDeg = 0.0f;
    GimbalYaw_TuneResetRequest = 0U;
    s_small_yaw_speed_filtered = 0.0f;

    /* 大yaw6020 */
    GimbalAxis_SetEffort(&s_big_yaw_axis, 0.0f);

    PID_InitPIDParam(&gimbalyaw->spdPIDParam, Const_GimbalYawSpdParam[0][0], Const_GimbalYawSpdParam[0][1], Const_GimbalYawSpdParam[0][2], Const_GimbalYawSpdParam[0][3], 
                    Const_GimbalYawSpdParam[0][4], Const_GimbalYawSpdParam[1][0], Const_GimbalYawSpdParam[1][1], Const_GimbalYawSpdParam[2][0], Const_GimbalYawSpdParam[2][1], 
                    Const_GimbalYawSpdParam[3][0], Const_GimbalYawSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&gimbalyaw->angPIDParam, Const_GimbalYawAngParam[0][0], Const_GimbalYawAngParam[0][1], Const_GimbalYawAngParam[0][2], Const_GimbalYawAngParam[0][3], 
                    Const_GimbalYawAngParam[0][4], Const_GimbalYawAngParam[1][0], Const_GimbalYawAngParam[1][1], Const_GimbalYawAngParam[2][0], Const_GimbalYawAngParam[2][1], 
                    Const_GimbalYawAngParam[3][0], Const_GimbalYawAngParam[3][1], PID_POSITION);                     

    /* Small yaw uses IMU attitude and rate feedback with dedicated gains. */
    PID_InitPIDParam(&gimbalyaw->GimbalSmallYaw_spdPIDParam, Const_GimbalSmallYawSpdParam[0][0], Const_GimbalSmallYawSpdParam[0][1], Const_GimbalSmallYawSpdParam[0][2], Const_GimbalSmallYawSpdParam[0][3],
                    Const_GimbalSmallYawSpdParam[0][4], Const_GimbalSmallYawSpdParam[1][0], Const_GimbalSmallYawSpdParam[1][1], Const_GimbalSmallYawSpdParam[2][0], Const_GimbalSmallYawSpdParam[2][1],
                    Const_GimbalSmallYawSpdParam[3][0], Const_GimbalSmallYawSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&gimbalyaw->GimbalSmallYaw_angPIDParam, Const_GimbalSmallYawAngParam[0][0], Const_GimbalSmallYawAngParam[0][1], Const_GimbalSmallYawAngParam[0][2], Const_GimbalSmallYawAngParam[0][3],
                    Const_GimbalSmallYawAngParam[0][4], Const_GimbalSmallYawAngParam[1][0], Const_GimbalSmallYawAngParam[1][1], Const_GimbalSmallYawAngParam[2][0], Const_GimbalSmallYawAngParam[2][1],
                    Const_GimbalSmallYawAngParam[3][0], Const_GimbalSmallYawAngParam[3][1], PID_POSITION);
}

/**
  * @brief      Get the pointer of gimbal control object
  * @param      NULL
  * @retval     Pointer to gimbal control object
  */
GimbalPitch_GimbalPitchTypeDef* GimbalPitch_GetGimbalPitchPtr() {
    return &GimbalPitch_gimbalPitchControlData;
}
/**
  * @brief      Get the pointer of gimbal control object
  * @param      NULL
  * @retval     Pointer to gimbal control object
  */
GimbalYaw_GimbalYawTypeDef* GimbalYaw_GetGimbalYawPtr() {
    return &GimbalYaw_gimbalYawControlData;
}

/** 
 * @brief 读取云台“大 yaw 电机(6020)”编码器的原始角度（0~360°）。
 * @note  这个角度是电机编码器直接给出的 limited_angle，未做任何“朝向对齐/零位校准”。
 *        如果机械正前方对应的原始角不是 0°，这里读出来就会带一个固定偏置。
 *        这个偏置需要在 `sys_const.c` 里通过 `Const_GimbalBigYawAngleOffset` 记录。
 */
float GimbalYaw_GetEncoderRawDeg(void) {
    return GimbalAxis_GetFeedback(&s_big_yaw_axis).position;
}

/**
 * @brief 读取云台“大 yaw 电机(6020)”的“逻辑角/校准角”（0~360°）。
 * @note  逻辑角 = 原始角 - 零位偏置(Const_GimbalBigYawAngleOffset)，再归一化到 [0, 360)。
 *        含义是：当你把云台物理朝向“定义的正前方”时，把当时的原始角填进 offset，
 *        之后 `GimbalYaw_GetEncoderLogicalDeg()` 在“正前方”就会返回接近 0°。
 *        这样做是为了让控制/调参使用一个与“正前方对齐”的角度坐标系，而不是编码器原始坐标系。
 */
float GimbalYaw_GetEncoderLogicalDeg(void) {
    return GimbalYaw_Normalize360(GimbalAxis_GetLogicalPosition(&s_big_yaw_axis));
}

float GimbalPitch_GetPositionFeedback(void) {
    return GimbalAxis_GetFeedback(&s_pitch_axis).position;
}

/**
  * @brief      Set the gimbal control output calculation enabled state
  * @param      state: Enabled, 1 is enabled, 0 is disabled
  * @retval     NULL
  */
void GimbalPitch_SetGimbalPitchControlState(uint8_t state) {
    GimbalPitch_GimbalPitchTypeDef *gimbalPitch = GimbalPitch_GetGimbalPitchPtr();

    gimbalPitch->control_state = state;
}
/**
  * @brief      Set the gimbal control output calculation enabled state
  * @param      state: Enabled, 1 is enabled, 0 is disabled
  * @retval     NULL
  */
void GimbalYaw_SetGimbalYawControlState(uint8_t state) {
    GimbalYaw_GimbalYawTypeDef *gimbalYaw = GimbalYaw_GetGimbalYawPtr();

    gimbalYaw->control_state = state;
}


/**
  * @brief      Set gimbal control output enable status
  * @param      state: Enabled, 1 is enabled, 0 is disabled
  * @retval     NULL
  */
void GimbalPitch_SetGimbalPitchOutputState(uint8_t state) {
    GimbalPitch_GimbalPitchTypeDef *gimbalPitch = GimbalPitch_GetGimbalPitchPtr();

    gimbalPitch->output_state = state;
}
/**
  * @brief      Set gimbal control output enable status
  * @param      state: Enabled, 1 is enabled, 0 is disabled
  * @retval     NULL
  */
void GimbalYaw_SetGimbalYawOutputState(uint8_t state) {
    GimbalYaw_GimbalYawTypeDef *gimbalYaw = GimbalYaw_GetGimbalYawPtr();

    gimbalYaw->output_state = state;
}



/**
  * @brief      Set the target value of gimbal pitch
  * @param      pitch_ref: gimbal pitch target value
  * @retval     NULL
  */
void GimbalPitch_SetPitchRef(float pitch_ref) {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();

    gimbalpitch->pitch_ref += pitch_ref;
    if (gimbalpitch->limits_initialized != 0U) {
        gimbalpitch->pitch_ref = Gimbal_Clamp(
            gimbalpitch->pitch_ref,
            gimbalpitch->imu_min_ref,
            gimbalpitch->imu_max_ref);
    }
}
/**
  * @brief      Set the target value of gimbal yaw
  * @param      yaw_ref: gimbal yaw target value
  * @retval     NULL
  */
void GimbalYaw_SetYawRef(float yaw_ref) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
    
    gimbalyaw->yaw_ref = yaw_ref;
}                                                             //Pitch??????Yaw?????

void GimbalYaw_AddYawRef(float yaw_step_deg) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
    gimbalyaw->yaw_ref += yaw_step_deg;
}

void GimbalYaw_AddSmallYawRef(float yaw_step_deg) {
    GimbalYaw_AddYawRef(yaw_step_deg);
}

void GimbalYaw_SetSmallYawRateDps(float rate_dps) {
    uint32_t primask = __get_PRIMASK();
    /* Control and gimbal tasks must publish/read rate and timestamp together. */
    __disable_irq();
    SmallYawRate_SetDps(&s_small_yaw_rate_command, rate_dps, HAL_GetTick());
    __set_PRIMASK(primask);
}

static float GimbalYaw_GetSmallYawRateRpm(void) {
    uint32_t primask = __get_PRIMASK();
    float rpm;
    __disable_irq();
    rpm = SmallYawRate_GetRpm(&s_small_yaw_rate_command, HAL_GetTick());
    __set_PRIMASK(primask);
    return rpm;
}

/**
* @brief      Pitch轴角度限位
* @param      ref:输入的俯仰角目标值
* @retval     限位后的俯仰角目标值
*/
float Gimbal_LimitPitch(float ref) {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();

    if (gimbalpitch->limits_initialized == 0U) return 0.0f;
/*如果当前俯仰角已经超过向上最大阈值，且输入的目标值 ref > 0（想继续向上转），则触发限位
如果当前俯仰角已经低于向下最大阈值，且用户输入的目标值 ref < 0（想继续向下转），则触发限位*/
    if (((gimbalpitch->motor_ref >= gimbalpitch->motor_max_ref) &&
         (ref * GimbalPitch_ImuDirection > 0.0f)) ||
        ((gimbalpitch->motor_ref <= gimbalpitch->motor_min_ref) &&
         (ref * GimbalPitch_ImuDirection < 0.0f)))
        return 0.0f;
        // Out of depression set maximum ref
    else return ref;
}


/**
* @brief      Yaw angle limit
* @param      ref: Yaw set ref
* @retval     Limited ywa ref
*/
float Gimbal_LimitYaw(float ref) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();

	if (buscomm->cha_mode  == Cha_Gyro)
        return ref;
	else if (((ins->YawTotalAngle - buscomm->yaw_ref < -Const_YAW_MAXANGLE) && (ref > 0)) || 
             ((ins->YawTotalAngle - buscomm->yaw_ref >  Const_YAW_MAXANGLE) && (ref < 0))) 
        return 0.0f;
    else return ref;
}

/**
* @brief      Set pitch ref
* @param      ref: Yaw set ref
* @retval     NULL
*/
void Gimbal_SetPitchRef(float ref) {
    GimbalPitch_SetPitchRef(ref);
}

/**
  * @brief      Setting IMU yaw position feedback
  * @param      imu_yaw_position_fdb: IMU Yaw Position feedback
  * @retval     NULL
  */
void GimbalYaw_SetIMUYawPositionFdb(float imu_yaw_position_fdb) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->yaw_position_fdb = imu_yaw_position_fdb;
}


/**
  * @brief      Setting IMU yaw speed feedback
  * @param      imu_yaw_speed_fdb: IMU Yaw Speed feedback
  * @retval     NULL
  */
void GimbalYaw_SetIMUYawSpeedFdb(float imu_yaw_speed_fdb) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->yaw_speed_fdb = imu_yaw_speed_fdb;
}

float TESTimu_error;
/**
  * @brief      Control function of gimbal pitch
  * @param      NULL
  * @retval     NULL
  */
void GimbalPitch_Control() {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();
    Actuator_FeedbackTypeDef feedback = GimbalAxis_GetFeedback(&s_pitch_axis);
    float attitude_error;
    float motor_rate_ref;
    float rate_limit;
    float gravity_effort;
    uint8_t imu_ready = GimbalImu_IsReady();

    GimbalPitch_DiagMotorPosition = feedback.position;
    GimbalPitch_DiagMotorSpeed = feedback.velocity;
    GimbalPitch_DiagMotorEffort = feedback.effort;
    GimbalPitch_DiagMotorOnline = feedback.online;

    if (imu_ready != 0U) {
        gimbalpitch->imu_position_fdb =
            ins->Roll * PI / 180.0f + Const_PITCH_MOTOR_INIT_OFFSETf;
        gimbalpitch->imu_speed_fdb = ins->Gyro[Y_INS];
    }

    if (feedback.online == 0U ||
        (gimbalpitch->limits_initialized != 0U &&
         fabsf(feedback.position - gimbalpitch->motor_zero_ref) >
             GIMBAL_PITCH_MOTOR_REV_RAD * 0.5f)) {
        gimbalpitch->limits_initialized = 0U;
        s_pitch_was_enabled = 0U;
    }
    if (gimbalpitch->limits_initialized == 0U &&
        imu_ready != 0U && feedback.online != 0U) {
        GimbalPitch_InitializeMechanicalLimits(
            gimbalpitch, feedback.position, gimbalpitch->imu_position_fdb);
    }

    if (gimbalpitch->limits_initialized != 0U &&
        imu_ready != 0U && feedback.online != 0U) {
        float motor_to_min = gimbalpitch->motor_min_ref - feedback.position;
        float motor_to_max = gimbalpitch->motor_max_ref - feedback.position;

        /* Convert remaining mechanical travel into the current world-angle frame. */
        if (GimbalPitch_ImuDirection >= 0.0f) {
            gimbalpitch->imu_min_ref =
                gimbalpitch->imu_position_fdb + motor_to_min;
            gimbalpitch->imu_max_ref =
                gimbalpitch->imu_position_fdb + motor_to_max;
        } else {
            gimbalpitch->imu_min_ref =
                gimbalpitch->imu_position_fdb - motor_to_max;
            gimbalpitch->imu_max_ref =
                gimbalpitch->imu_position_fdb - motor_to_min;
        }
    }

    GimbalPitch_TuneImuKp = Gimbal_Clamp(
        GimbalPitch_TuneImuKp, 0.0f, 20.0f);
    GimbalPitch_TuneImuKd = Gimbal_Clamp(
        GimbalPitch_TuneImuKd, 0.0f, 5.0f);
    rate_limit = Gimbal_Clamp(
        GimbalPitch_TuneRateLimitRadS, 0.0f, 3.0f);
    GimbalPitch_TuneRateLimitRadS = rate_limit;
    GimbalPitch_TuneGravityEffort = Gimbal_Clamp(
        GimbalPitch_TuneGravityEffort, -2.0f, 2.0f);
    GimbalPitch_TuneAngleDeadbandRad = Gimbal_Clamp(
        GimbalPitch_TuneAngleDeadbandRad, 0.0f, 0.05f);
    GimbalPitch_TuneRateDeadbandRadS = Gimbal_Clamp(
        GimbalPitch_TuneRateDeadbandRadS, 0.0f, 0.5f);

    if (GimbalPitch_TuneResetRequest != 0U) {
        s_pitch_was_enabled = 0U;
        GimbalPitch_DiagAppliedStepRad = 0.0f;
        GimbalPitch_TuneResetRequest = 0U;
    }

    if (gimbalpitch->control_state != 1U ||
        gimbalpitch->output_state != 1U ||
        GimbalPitch_ImuEnable == 0U ||
        gimbalpitch->limits_initialized == 0U ||
        imu_ready == 0U ||
        feedback.online == 0U) {
        s_pitch_was_enabled = 0U;
        gimbalpitch->imu_error = 0.0f;
        gimbalpitch->motor_rate_ref = 0.0f;
        GimbalPitch_DiagGravityEffort = 0.0f;
        GimbalPitch_TuneStepRad = 0.0f;
        GimbalAxis_SetEffort(&s_pitch_axis, 0.0f);
        if (feedback.online != 0U) {
            gimbalpitch->motor_ref = feedback.position;
            GimbalAxis_SetPosition(&s_pitch_axis, feedback.position);
        }
        return;
    }

    if (s_pitch_was_enabled == 0U) {
        gimbalpitch->pitch_ref = gimbalpitch->imu_position_fdb;
        gimbalpitch->pitch_ref_smooth = gimbalpitch->pitch_ref;
        gimbalpitch->motor_ref = feedback.position;
        s_pitch_was_enabled = 1U;
    }

    gimbalpitch->pitch_ref_smooth += gimbalpitch->filter_alpha *
                                     (gimbalpitch->pitch_ref - gimbalpitch->pitch_ref_smooth);

    if (GimbalPitch_TuneStepRad != 0.0f) {
        float step = Gimbal_Clamp(GimbalPitch_TuneStepRad, -0.01f, 0.01f);
        /* Commissioning steps are relative to the measured pose, not a stale target. */
        gimbalpitch->pitch_ref = Gimbal_Clamp(
            gimbalpitch->imu_position_fdb + step,
            gimbalpitch->imu_min_ref,
            gimbalpitch->imu_max_ref);
        gimbalpitch->pitch_ref_smooth = gimbalpitch->pitch_ref;
        GimbalPitch_DiagAppliedStepRad =
            gimbalpitch->pitch_ref - gimbalpitch->imu_position_fdb;
        GimbalPitch_TuneStepRad = 0.0f;
    }

    attitude_error = Gimbal_ApplyDeadband(
        gimbalpitch->pitch_ref_smooth - gimbalpitch->imu_position_fdb,
        GimbalPitch_TuneAngleDeadbandRad);
    gimbalpitch->imu_speed_fdb = Gimbal_ApplyDeadband(
        gimbalpitch->imu_speed_fdb,
        GimbalPitch_TuneRateDeadbandRadS);
    gimbalpitch->imu_error = attitude_error;
    TESTimu_error = attitude_error;
    motor_rate_ref = GimbalPitch_ImuDirection *
        (GimbalPitch_TuneImuKp * attitude_error -
         GimbalPitch_TuneImuKd * gimbalpitch->imu_speed_fdb);
    motor_rate_ref = Gimbal_Clamp(motor_rate_ref,
                                 -rate_limit,
                                 rate_limit);
    gimbalpitch->motor_rate_ref = motor_rate_ref;
    gimbalpitch->motor_ref += motor_rate_ref * GIMBAL_CONTROL_DT_S;
    gimbalpitch->motor_ref = Gimbal_Clamp(
        gimbalpitch->motor_ref,
        feedback.position - GIMBAL_PITCH_MOTOR_REF_LEAD_MAX_RAD,
        feedback.position + GIMBAL_PITCH_MOTOR_REF_LEAD_MAX_RAD);
    gimbalpitch->motor_ref = Gimbal_Clamp(gimbalpitch->motor_ref,
                                         gimbalpitch->motor_min_ref,
                                         gimbalpitch->motor_max_ref);
    gravity_effort = GimbalPitch_TuneGravityEffort *
                     arm_cos_f32(ins->Roll * PI / 180.0f);
    GimbalPitch_DiagGravityEffort = gravity_effort;
    GimbalAxis_SetPosition(&s_pitch_axis, gimbalpitch->motor_ref);
    GimbalAxis_SetEffort(&s_pitch_axis, gravity_effort);
}

void GimbalPitch_Output(void) {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();

    if (YawIdentApp_OwnsControl() && !YawIdentApp_AllowYaw())
        gimbalpitch->output_state = 0U;

    GimbalAxis_SetEnabled(&s_pitch_axis, gimbalpitch->output_state == 1U);
    RobotActuators_Service();
    if (gimbalpitch->output_state == 1U) GimbalAxis_Flush(&s_pitch_axis);
}
/**
  * @brief      IMU-stabilized small yaw plus demand-based encoder big yaw
  */
void GimbalYaw_Control() {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    Actuator_FeedbackTypeDef big_feedback;
    Actuator_FeedbackTypeDef small_feedback;
    float small_encoder_deg;
    float effort;
    float err;
    float fdb_for_pid;
    float limited;
    float direction = GimbalYaw_DiagSmallDirection;
    uint8_t imu_ready;
    GimbalYaw_MpcRequest mpc_request;
    YawMpc_Input mpc_input;
    YawMpc_Output mpc_output;

    /* Keep telemetry and control disabled even if a debugger sets the enable flag. */
    if (BIG_YAW_PASSIVE_SMALL_TEST != 0U) GimbalYaw_DiagSmallEnable = 0U;
    if (BIG_YAW_OUTPUT_INHIBIT_TEST != 0U) GimbalYaw_DiagBigEnable = 0U;
    /* Live sensor observations must not depend on motor enable or SAFE. */
    imu_ready = GimbalImu_IsReady();
    GimbalYaw_DiagSmallFdbDeg = imu_ready && isfinite(ins->YawTotalAngle) ?
        GimbalYaw_Normalize360(ins->YawTotalAngle) : 0.0f;
    GimbalYaw_DiagSmallRawSpeedRpm = imu_ready && isfinite(ins->Gyro[Z_INS]) ?
        ins->Gyro[Z_INS] * GIMBAL_RAD_TO_RPM : 0.0f;
    gimbalyaw->yaw_ref_limit_status = 0U;
    GimbalYaw_DiagSmallLimitsValid = 0U;
    GimbalYaw_DiagBigCurrent = Motor_Big_YawMotor.encoder.current;
    GimbalYaw_DiagBigTemperatureC = Motor_Big_YawMotor.encoder.temp;

    if (gimbalyaw->control_state != 1U || gimbalyaw->output_state != 1U) {
        YawMpc_Reset(&s_yaw_mpc, YAW_MPC_DISABLED);
        GimbalYaw_DiagMpcRequested = 0U;
        GimbalYaw_DiagMpcActive = 0U;
        GimbalYaw_DiagMpcUpdated = 0U;
        GimbalYaw_DiagMpcReason = YAW_MPC_DISABLED;
        GimbalYaw_DiagMpcBigRefDps = 0.0f;
        GimbalYaw_DiagMpcSmallRefDps = 0.0f;
        GimbalYaw_SetSmallYawRateDps(0.0f);
        GimbalYaw_TuneSmallStepDeg = 0.0f;
        GimbalYaw_StopBig(gimbalyaw);
        s_small_yaw_was_enabled = 0U;
        GimbalYaw_ClearBigController(gimbalyaw);
        GimbalYaw_ClearSmallController(gimbalyaw);
        GimbalYaw_DiagBigEffortCmd = 0.0f;
        GimbalYaw_DiagSmallEffortCmd = 0.0f;
        GimbalYaw_DiagSmallErrorDeg = 0.0f;
        GimbalYaw_DiagSmallSpeedRefRpm = 0.0f;
        GimbalYaw_DiagSmallSpeedRpm = 0.0f;
        GimbalYaw_DiagSmallCurrent = 0.0f;
        s_small_yaw_speed_filtered = 0.0f;
        GimbalAxis_SetEffort(&s_big_yaw_axis, 0.0f);
        GimbalAxis_SetEffort(&s_small_yaw_axis, 0.0f);
        return;
    }

    big_feedback = GimbalAxis_GetFeedback(&s_big_yaw_axis);
    small_feedback = GimbalAxis_GetFeedback(&s_small_yaw_axis);
    small_encoder_deg = 0.0f;
    GimbalYaw_DiagSmallLimitsValid = (uint8_t)(
        small_feedback.online != 0U &&
        Motor_IsMotorFeedbackFresh(&Motor_Small_YawMotor,
                                   YAW_LIMIT_FEEDBACK_MAX_AGE_MS) &&
        isfinite(small_feedback.velocity) &&
        isfinite(ins->YawTotalAngle) && isfinite(ins->Gyro[Z_INS]) &&
        (direction == 1.0f || direction == -1.0f) &&
        YawLimits_Position(small_feedback.position, &small_encoder_deg));
    GimbalYaw_DiagSmallJointDeg = small_encoder_deg;
    if (GimbalYaw_DiagSmallLimitsValid == 0U)
        gimbalyaw->yaw_ref_limit_status |= YAW_LIMIT_STATUS_INVALID;
    else if (small_encoder_deg < YAW_LIMIT_MIN_DEG ||
             small_encoder_deg > YAW_LIMIT_MAX_DEG)
        gimbalyaw->yaw_ref_limit_status |= YAW_LIMIT_STATUS_OUTSIDE;
    GimbalYaw_DiagSmallCurrent = small_feedback.effort;
    GimbalYaw_ApplySmallTune(gimbalyaw);

    mpc_request = GimbalYaw_ReadMpcRequest();
    memset(&mpc_input, 0, sizeof(mpc_input));
    mpc_input.now_ms = HAL_GetTick();
    mpc_input.request_ms = mpc_request.tick_ms;
    mpc_input.requested = (uint8_t)(mpc_request.enabled &&
        !YawIdentApp_OwnsControl());
    mpc_input.feedback_valid = (uint8_t)(
        GimbalYaw_DiagSmallEnable != 0U &&
        GimbalYaw_DiagSmallLimitsValid != 0U && imu_ready != 0U &&
        big_feedback.online != 0U &&
        Motor_IsMotorFeedbackFresh(&Motor_Big_YawMotor,
                                   YAW_LIMIT_FEEDBACK_MAX_AGE_MS) &&
        isfinite(big_feedback.position) && isfinite(big_feedback.velocity) &&
        direction == 1.0f && GimbalYaw_DiagBigDirection == 1.0f);
    mpc_input.heading_error_deg = GimbalYaw_Wrap180(
        gimbalyaw->yaw_ref - ins->YawTotalAngle);
    mpc_input.small_joint_deg = small_encoder_deg;
    mpc_input.big_rate_dps = big_feedback.velocity * 6.0f;
    mpc_input.small_inertial_rate_dps = ins->Gyro[Z_INS] * 57.295779513f;
    mpc_input.target_heading_rate_dps = mpc_request.target_rate_dps;
    mpc_input.previous_big_reference_dps = GimbalYaw_DiagBigSpeedRefRpm * 6.0f;
    mpc_input.previous_small_reference_dps = GimbalYaw_DiagSmallSpeedRefRpm * 6.0f;
    mpc_output = YawMpc_Update(&s_yaw_mpc, &mpc_input);
    GimbalYaw_DiagMpcRequested = mpc_input.requested;
    GimbalYaw_DiagMpcActive = mpc_output.active;
    GimbalYaw_DiagMpcUpdated = mpc_output.updated;
    GimbalYaw_DiagMpcReason = mpc_output.reason;
    GimbalYaw_DiagMpcBigRefDps = mpc_output.big_reference_dps;
    GimbalYaw_DiagMpcSmallRefDps = mpc_output.small_reference_dps;
    GimbalYaw_DiagMpcBigUnconstrainedDps = s_yaw_mpc.unconstrained[0];
    GimbalYaw_DiagMpcSmallUnconstrainedDps = s_yaw_mpc.unconstrained[1];

    if (GimbalYaw_DiagSmallEnable != 0U &&
        small_feedback.online != 0U &&
        GimbalYaw_DiagSmallLimitsValid != 0U &&
        imu_ready != 0U) {
        if (s_small_yaw_was_enabled == 0U) {
            if (!YawIdentApp_OwnsControl()) gimbalyaw->yaw_ref = ins->YawTotalAngle;
            if (buscomm != NULL) buscomm->yaw_ref = gimbalyaw->yaw_ref;
            s_small_yaw_speed_filtered = GimbalYaw_DiagSmallRawSpeedRpm;
            GimbalYaw_ClearSmallController(gimbalyaw);
            s_small_yaw_was_enabled = 1U;
        }

        s_small_yaw_speed_filtered += GimbalYaw_TuneSmallSpeedFilterAlpha *
            (GimbalYaw_DiagSmallRawSpeedRpm - s_small_yaw_speed_filtered);
        GimbalYaw_DiagSmallSpeedRpm = s_small_yaw_speed_filtered;

        if (YawIdentApp_UsesSpeedReference() || GimbalYaw_DiagMpcActive != 0U) {
            GimbalYaw_TuneSmallStepDeg = 0.0f;
            GimbalYaw_DiagSmallRefDeg = GimbalYaw_DiagSmallFdbDeg;
            GimbalYaw_DiagSmallErrorDeg = 0.0f;
            PID_ClearPID(&gimbalyaw->GimbalSmallYaw_angPID);
            GimbalYaw_DiagSmallSpeedRefRpm = YawIdentApp_UsesSpeedReference() ?
                YawIdentApp_SmallSpeedReferenceRpm() :
                GimbalYaw_DiagMpcSmallRefDps / 6.0f;
        } else if (SMALL_YAW_ANGLE_OPEN_TEST != 0U) {
            /* Angle targets and tuning steps cannot drive the rate-only test. */
            GimbalYaw_TuneSmallStepDeg = 0.0f;
            gimbalyaw->yaw_ref = ins->YawTotalAngle;
            if (buscomm != NULL) buscomm->yaw_ref = gimbalyaw->yaw_ref;
            GimbalYaw_DiagSmallRefDeg = GimbalYaw_DiagSmallFdbDeg;
            GimbalYaw_DiagSmallErrorDeg = 0.0f;
            PID_ClearPID(&gimbalyaw->GimbalSmallYaw_angPID);
            GimbalYaw_DiagSmallSpeedRefRpm = GimbalYaw_GetSmallYawRateRpm();
        } else {
            if (GimbalYaw_TuneSmallStepDeg != 0.0f) {
                float step = GimbalYaw_TuneSmallStepDeg;
                if (step > 10.0f) step = 10.0f;
                if (step < -10.0f) step = -10.0f;
                gimbalyaw->yaw_ref += step;
                GimbalYaw_TuneSmallStepDeg = 0.0f;
            }

            if (!isfinite(gimbalyaw->yaw_ref)) gimbalyaw->yaw_ref = ins->YawTotalAngle;
            err = GimbalYaw_Wrap180(gimbalyaw->yaw_ref - ins->YawTotalAngle);
            limited = YawLimits_Error(small_encoder_deg, err, direction);
            if (limited != err) {
                err = limited;
                gimbalyaw->yaw_ref = ins->YawTotalAngle + err;
                if (buscomm != NULL) buscomm->yaw_ref = gimbalyaw->yaw_ref;
                gimbalyaw->GimbalSmallYaw_angPID.sum = 0.0f;
                gimbalyaw->yaw_ref_limit_status |= YAW_LIMIT_STATUS_REFERENCE;
            }
            GimbalYaw_DiagSmallRefDeg = GimbalYaw_Normalize360(gimbalyaw->yaw_ref);
            GimbalYaw_DiagSmallErrorDeg = err;
            fdb_for_pid = gimbalyaw->yaw_ref - err;

            PID_SetPIDRef(&gimbalyaw->GimbalSmallYaw_angPID, gimbalyaw->yaw_ref);
            PID_SetPIDFdb(&gimbalyaw->GimbalSmallYaw_angPID, fdb_for_pid);
            PID_CalcPID(&gimbalyaw->GimbalSmallYaw_angPID,
                        &gimbalyaw->GimbalSmallYaw_angPIDParam);
            GimbalYaw_DiagSmallSpeedRefRpm =
                PID_GetPIDOutput(&gimbalyaw->GimbalSmallYaw_angPID);
        }

        limited = YawLimits_Speed(small_encoder_deg,
            GimbalYaw_DiagSmallSpeedRefRpm, GimbalYaw_DiagSmallRawSpeedRpm,
            small_feedback.velocity, direction, GimbalYaw_TuneSmallSpeedLimitRpm);
        if (limited != GimbalYaw_DiagSmallSpeedRefRpm) {
            GimbalYaw_DiagSmallSpeedRefRpm = limited;
            gimbalyaw->GimbalSmallYaw_angPID.sum = 0.0f;
            gimbalyaw->GimbalSmallYaw_spdPID.sum = 0.0f;
            gimbalyaw->yaw_ref_limit_status |= YAW_LIMIT_STATUS_SPEED;
        }

        PID_SetPIDRef(&gimbalyaw->GimbalSmallYaw_spdPID,
                      GimbalYaw_DiagSmallSpeedRefRpm);
        PID_SetPIDFdb(&gimbalyaw->GimbalSmallYaw_spdPID,
                      GimbalYaw_DiagSmallSpeedRpm);
        PID_CalcPID(&gimbalyaw->GimbalSmallYaw_spdPID,
                    &gimbalyaw->GimbalSmallYaw_spdPIDParam);

        effort = GimbalYaw_DiagSmallDirection *
                 PID_GetPIDOutput(&gimbalyaw->GimbalSmallYaw_spdPID);
        limited = YawLimits_Effort(small_encoder_deg, effort);
        if (limited != effort) {
            effort = limited;
            gimbalyaw->GimbalSmallYaw_spdPID.sum = 0.0f;
            gimbalyaw->yaw_ref_limit_status |= YAW_LIMIT_STATUS_EFFORT;
        }
        GimbalYaw_DiagSmallEffortCmd = GimbalYaw_ClampEffort(effort);
        GimbalAxis_SetEffort(&s_small_yaw_axis, GimbalYaw_DiagSmallEffortCmd);
    } else {
        s_small_yaw_was_enabled = 0U;
        GimbalYaw_SetSmallYawRateDps(0.0f);
        GimbalYaw_ClearSmallController(gimbalyaw);
        GimbalYaw_DiagSmallEffortCmd = 0.0f;
        GimbalYaw_DiagSmallErrorDeg = 0.0f;
        GimbalYaw_DiagSmallSpeedRefRpm = 0.0f;
        GimbalYaw_DiagSmallSpeedRpm = 0.0f;
        s_small_yaw_speed_filtered = 0.0f;
        GimbalYaw_TuneSmallStepDeg = 0.0f;
        GimbalAxis_SetEffort(&s_small_yaw_axis, 0.0f);
    }

    GimbalYaw_ControlBig(gimbalyaw, &big_feedback,
        small_encoder_deg, small_feedback.velocity);
    if (GimbalYaw_DiagMpcActive != 0U &&
        GimbalYaw_DiagBigActive != 0U && s_small_yaw_was_enabled != 0U) {
        YawMpc_CommitApplied(&s_yaw_mpc,
            GimbalYaw_DiagBigSpeedRefRpm * 6.0f,
            GimbalYaw_DiagSmallSpeedRefRpm * 6.0f);
        GimbalYaw_DiagMpcBigRefDps = s_yaw_mpc.output[0];
        GimbalYaw_DiagMpcSmallRefDps = s_yaw_mpc.output[1];
    } else if (GimbalYaw_DiagMpcActive != 0U) {
        YawMpc_Reset(&s_yaw_mpc, YAW_MPC_INVALID_FEEDBACK);
        GimbalYaw_DiagMpcActive = 0U;
        GimbalYaw_DiagMpcReason = YAW_MPC_INVALID_FEEDBACK;
        GimbalYaw_DiagMpcBigRefDps = 0.0f;
        GimbalYaw_DiagMpcSmallRefDps = 0.0f;
    }
}

void GimbalYaw_Output(void) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
    YawIdentApp_ValidateOutput();
    uint8_t identifying = YawIdentApp_OwnsControl();
    uint8_t ident_allow = identifying && YawIdentApp_AllowYaw();
    uint32_t now = HAL_GetTick();
    BigYawManual_Command command = GimbalYaw_ReadBigManualCommand();
    Actuator_FeedbackTypeDef feedback = GimbalAxis_GetFeedback(&s_big_yaw_axis);
    uint8_t manual_allow = (uint8_t)(!identifying && GimbalYaw_BigManualPermitted(now) &&
        s_big_yaw_manual.active && command.generation == s_big_yaw_manual.generation &&
        BigYawManual_Fresh(&command, now) && feedback.online &&
        Motor_IsMotorFeedbackFresh(&Motor_Big_YawMotor,
                                   YAW_LIMIT_FEEDBACK_MAX_AGE_MS) &&
        isfinite(feedback.position) && isfinite(feedback.velocity) &&
        gimbalyaw->control_state == 1U && gimbalyaw->output_state == 1U &&
        s_small_yaw_was_enabled && GimbalImu_IsReady() &&
        GimbalYaw_DiagSmallLimitsValid && GimbalYaw_DiagBigDirection == 1.0f);
    uint8_t mpc_allow = (uint8_t)(!identifying &&
        GimbalYaw_DiagMpcActive != 0U && GimbalYaw_DiagBigActive != 0U &&
        feedback.online &&
        Motor_IsMotorFeedbackFresh(&Motor_Big_YawMotor,
                                   YAW_LIMIT_FEEDBACK_MAX_AGE_MS) &&
        isfinite(feedback.position) && isfinite(feedback.velocity) &&
        gimbalyaw->control_state == 1U && gimbalyaw->output_state == 1U &&
        s_small_yaw_was_enabled && GimbalImu_IsReady() &&
        GimbalYaw_DiagSmallLimitsValid && GimbalYaw_DiagBigDirection == 1.0f);
    uint8_t allow;
    allow = (uint8_t)(ident_allow || manual_allow || mpc_allow);
    if (!identifying && s_big_yaw_manual.active && !manual_allow)
        GimbalYaw_StopBig(gimbalyaw);
    if (identifying && !ident_allow) gimbalyaw->output_state = 0U;
    if (BIG_YAW_OUTPUT_INHIBIT_TEST != 0U && !allow) {
        GimbalAxis_SetEnabled(&s_big_yaw_axis, 0U);
        GimbalAxis_SetEffort(&s_big_yaw_axis, 0.0f);
    }
    if (BIG_YAW_PASSIVE_SMALL_TEST != 0U)
        GimbalAxis_SetEffort(&s_small_yaw_axis, 0.0f);
    if (gimbalyaw->output_state != 1) {
        GimbalAxis_SetEffort(&s_big_yaw_axis, 0.0f);
        GimbalAxis_SetEffort(&s_small_yaw_axis, 0.0f);
    }
    GimbalAxis_Flush(&s_big_yaw_axis);
    GimbalAxis_Flush(&s_small_yaw_axis);
}


///**
//  * @brief      Gimbal yaw output function
//  * @param      NULL
//  * @retval     NULL
//  */
//void GimbalYaw_Output() {
//    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

//    if (gimbalyaw->output_state != 1) {
//		Motor_SetMotorOutput(&Motor_YawMotor, 0);
//	}
//    Motor_SendMotorGroupOutput(&Motor_YawMotors);
//}
