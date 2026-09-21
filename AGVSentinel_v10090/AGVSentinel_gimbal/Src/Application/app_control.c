#include "app_control.h"

#include "app_board_config.h"
#include "app_communicate.h"
#include <string.h>
#include "sys_dwt.h"

volatile float Control_DiagDtS;

#if BOARD_HAS_CHASSIS
#include "module_chassis.h"
#endif

#if BOARD_HAS_GIMBAL
#include "task.h"
#include "app_autoaim.h"
#include "app_chassis_frame.h"
#include "app_ins.h"
#include "app_yaw_backend.h"
#include "periph_pc_comm.h"
#include "periph_remote.h"
#include "module_gimbal.h"
#include "app_yaw_identification.h"
#include "module_yaw_limits.h"
#include "module_shoot.h"
#include "protocol_common.h"
#include "sys_const.h"
#endif

#define CONTROL_TASK_PERIOD_MS                  2U
#define CONTROL_RC_YAW_CDEG_S_PER_COUNT       (-20.0f)
#define CONTROL_RC_PITCH_MRAD_S_PER_COUNT       (2.0f)
#define CONTROL_RC_CHASSIS_OVERRIDE_DEADBAND     10
#define CONTROL_VISION_TIMEOUT_MS                100U
#define CONTROL_VISION_YAW_GAIN                  (0.004f)
#define CONTROL_VISION_YAW_DEADBAND_DEG          (0.02f)
#define CONTROL_VISION_YAW_FILTER_ALPHA          (0.20f)
#define CONTROL_VISION_PITCH_GAIN                (0.002f)

#define CONTROL_GIMBAL_FLAG_ENABLE   (1U << 0)
#define CONTROL_GIMBAL_FLAG_AUTOAIM  (1U << 1)
#define CONTROL_GIMBAL_FLAG_SHOOTER  (1U << 2)
#define CONTROL_GIMBAL_FLAG_FIRE     (1U << 3)
#define CONTROL_GIMBAL_FLAG_MPC      (1U << 4)

typedef struct {
    int16_t yaw_rate_cdeg_s;
    int16_t pitch_rate_mrad_s;
    uint8_t flags;
    uint8_t mode;
    uint8_t legacy_yaw_pid;
} Control_GimbalTargetTypeDef;

static float Control_ClampF32(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int16_t Control_ClampI16(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)value;
}

#if BOARD_HAS_CHASSIS
static void Control_ChassisStop(void)
{
    Chassis_SetChassisMode(Chassis_NULL);
    Chassis_SetChassisRef(0.0f, 0.0f, 0.0f);
}

static void Control_ChassisExecutorStep(void)
{
    const Comm_ChassisCommandTypeDef *command =
        Comm_BoardLinkGetChassisCommand();

    if (!Comm_BoardLinkChassisCommandOnline() ||
        command->mode > COMM_CHASSIS_MODE_GIMBAL) {
        Control_ChassisStop();
        return;
    }

    if (command->mode == COMM_CHASSIS_MODE_GIMBAL) {
        if (!Comm_BoardLinkGimbalFeedbackOnline()) {
            Control_ChassisStop();
            return;
        }
        Chassis_SetGimbalYaw(Comm_BoardLinkGimbalRelYawDeg());
    }

    Chassis_SetChassisMode((Chassis_ModeEnum)command->mode);
    Chassis_SetChassisRef((float)command->vx_ref,
                          (float)command->vy_ref,
                          (float)command->wz_ref);
}
#endif

#if BOARD_HAS_GIMBAL
static Control_GimbalTargetTypeDef s_gimbal_target;
static float s_vision_yaw_step;
static uint8_t s_gimbal_was_enabled;

static void Control_StopChassisTarget(Comm_ChassisCommandTypeDef *command)
{
    command->vx_ref = 0;
    command->vy_ref = 0;
    command->wz_ref = 0;
    command->mode = COMM_CHASSIS_MODE_STOP;
}

static uint8_t Control_SelectLatestPcChassisCommand(uint8_t *cmd_id)
{
    uint32_t chassis_time;
    uint32_t navigation_time;
    uint8_t chassis_online = PC_Comm_IsCommandOnline(PC_CMD_CHASSIS);
    uint8_t navigation_online = PC_Comm_IsCommandOnline(PC_CMD_NAVIGATION);

    if (!chassis_online && !navigation_online) return 0U;

    chassis_time = PC_Comm_GetCommandLastUpdate(PC_CMD_CHASSIS);
    navigation_time = PC_Comm_GetCommandLastUpdate(PC_CMD_NAVIGATION);
    *cmd_id = (chassis_online && (!navigation_online || chassis_time >= navigation_time))
                  ? PC_CMD_CHASSIS
                  : PC_CMD_NAVIGATION;
    return 1U;
}

static void Control_FillPcChassisTarget(
    uint8_t cmd_id, Comm_ChassisCommandTypeDef *target)
{
    PC_ParsedData_t *pc = PC_Comm_GetParsedDataPtr();
    float vx;
    float vy;
    float wz;

    if (cmd_id == PC_CMD_CHASSIS) {
        if (pc->control.chassis.mode == 0U) {
            Control_StopChassisTarget(target);
            return;
        }
        vx = (float)pc->control.chassis.vx_mm_s / 1000.0f;
        vy = (float)pc->control.chassis.vy_mm_s / 1000.0f;
        wz = (float)pc->control.chassis.wz_mrad_s / 1000.0f;
        target->mode = pc->control.chassis.mode == 2U
                           ? COMM_CHASSIS_MODE_GIMBAL
                           : COMM_CHASSIS_MODE_BODY;
    } else {
        vx = pc->control.navigation.linear_x;
        vy = pc->control.navigation.linear_y;
        wz = pc->control.navigation.angular_z;
        target->mode = COMM_CHASSIS_MODE_BODY;
    }

    target->vx_ref = Control_ClampI16(vx * Const_Chasiss_Navigate_linear_x);
    target->vy_ref = Control_ClampI16(-vy * Const_Chasiss_Navigate_linear_y);
    target->wz_ref = Control_ClampI16(wz * Const_Chasiss_Navigate_angular_z);
}

static uint8_t Control_FillRemoteSmallYawChassisTarget(
    const Remote_RemoteDataTypeDef *remote,
    Comm_ChassisCommandTypeDef *target)
{
    float body_vx;
    float body_vy;

    if (remote == NULL || target == NULL ||
        GimbalYaw_DiagBigActive == 0U ||
        GimbalYaw_DiagSmallLimitsValid == 0U ||
        !isfinite(REMOTE_CHASSIS_VX_GAIN) ||
        !isfinite(REMOTE_CHASSIS_VY_GAIN) ||
        REMOTE_CHASSIS_VX_GAIN == 0.0f ||
        REMOTE_CHASSIS_VY_GAIN == 0.0f ||
        !ChassisFrame_SmallYawToBody(
            (float)remote->remote.ch[1] * REMOTE_CHASSIS_VX_GAIN,
            (float)remote->remote.ch[0] * REMOTE_CHASSIS_VY_GAIN,
            Comm_BoardLinkGimbalRelYawDeg(), GimbalYaw_DiagSmallJointDeg,
            CHASSIS_GIMBAL_YAW_STEER_SIGN, &body_vx, &body_vy)) {
        if (target != NULL) Control_StopChassisTarget(target);
        return 0U;
    }

    /* BODY mode applies its axis gains on the chassis board. Divide them out
     * after rotating physical components so unequal future gains remain exact. */
    target->vx_ref = Control_ClampI16(body_vx / REMOTE_CHASSIS_VX_GAIN);
    target->vy_ref = Control_ClampI16(body_vy / REMOTE_CHASSIS_VY_GAIN);
    target->wz_ref = 0;
    target->mode = COMM_CHASSIS_MODE_BODY;
    return 1U;
}

static void Control_UpdateTargets(void)
{
    Remote_RemoteDataTypeDef *remote = Remote_GetRemoteDataPtr();
    Comm_ChassisCommandTypeDef chassis_target = {0};
    Control_GimbalTargetTypeDef gimbal_target = {0};
    uint8_t pc_chassis_cmd = 0U;
    uint8_t remote_online = Comm_BoardLinkRemoteOnline();
    uint8_t pc_motion_online =
        Control_SelectLatestPcChassisCommand(&pc_chassis_cmd);
    uint8_t pc_gimbal_online = PC_Comm_IsCommandOnline(PC_CMD_GIMBAL);
    Control_ModeEnum mode = CONTROL_MODE_SAFE;

    if (remote_online) {
        if (remote->remote.s[1] == Remote_SWITCH_DOWN) {
            mode = CONTROL_MODE_MANUAL;
        } else if (remote->remote.s[1] == Remote_SWITCH_MIDDLE) {
            mode = CONTROL_MODE_AUTO;
        }
    } else if (pc_motion_online || pc_gimbal_online) {
        mode = CONTROL_MODE_AUTO;
    }

    gimbal_target.mode = (uint8_t)mode;
    gimbal_target.legacy_yaw_pid = (uint8_t)(
        remote_online && remote->remote.s[0] == Remote_SWITCH_DOWN);
    Control_StopChassisTarget(&chassis_target);

    if (mode == CONTROL_MODE_MANUAL) {
        chassis_target.vx_ref = remote->remote.ch[1];
        chassis_target.vy_ref = remote->remote.ch[0];
        chassis_target.wz_ref = 0;
        chassis_target.mode = COMM_CHASSIS_MODE_GIMBAL;

        gimbal_target.yaw_rate_cdeg_s = Control_ClampI16(
            (float)remote->remote.ch[2] * CONTROL_RC_YAW_CDEG_S_PER_COUNT);
        gimbal_target.pitch_rate_mrad_s = Control_ClampI16(
            (float)remote->remote.ch[3] * CONTROL_RC_PITCH_MRAD_S_PER_COUNT);
        gimbal_target.flags = CONTROL_GIMBAL_FLAG_ENABLE;
        if (remote->mouse.r) {
            gimbal_target.flags |= CONTROL_GIMBAL_FLAG_AUTOAIM;
        }
    } else if (mode == CONTROL_MODE_AUTO) {
        gimbal_target.flags |= CONTROL_GIMBAL_FLAG_MPC;
        if (pc_motion_online) {
            Control_FillPcChassisTarget(pc_chassis_cmd, &chassis_target);
        }
        if (remote_online && ChassisFrame_RemoteTranslationActive(
                remote->remote.ch[1], remote->remote.ch[0],
                CONTROL_RC_CHASSIS_OVERRIDE_DEADBAND)) {
            /* Operator translation overrides an online PC motion command.
             * Resolve it here so MIDDLE follows the complete small-yaw heading
             * (calibrated big yaw plus small joint), then send a body-frame
             * command and avoid a second rotation on the chassis board. */
            (void)Control_FillRemoteSmallYawChassisTarget(remote,
                                                          &chassis_target);
        }

        if (remote_online || pc_gimbal_online) {
            gimbal_target.flags |= CONTROL_GIMBAL_FLAG_ENABLE;
        }
        if (remote_online) {
            gimbal_target.yaw_rate_cdeg_s = Control_ClampI16(
                (float)remote->remote.ch[2] * CONTROL_RC_YAW_CDEG_S_PER_COUNT);
            gimbal_target.pitch_rate_mrad_s = Control_ClampI16(
                (float)remote->remote.ch[3] * CONTROL_RC_PITCH_MRAD_S_PER_COUNT);
            gimbal_target.flags |= CONTROL_GIMBAL_FLAG_AUTOAIM;
        }
        if (pc_gimbal_online) {
            PC_Recv_Gimbal_t *pc_gimbal =
                &PC_Comm_GetParsedDataPtr()->control.gimbal;
            gimbal_target.yaw_rate_cdeg_s = pc_gimbal->yaw_rate_cdeg_s;
            gimbal_target.pitch_rate_mrad_s = pc_gimbal->pitch_rate_mrad_s;
            gimbal_target.mode = pc_gimbal->mode;
            if (pc_gimbal->fire) {
                gimbal_target.flags |=
                    CONTROL_GIMBAL_FLAG_SHOOTER | CONTROL_GIMBAL_FLAG_FIRE;
            }
            if ((pc_gimbal->mode & PC_GIMBAL_MODE_ENABLE) == 0U) {
                gimbal_target.flags &=
                    (uint8_t)~CONTROL_GIMBAL_FLAG_ENABLE;
            }
            if ((pc_gimbal->mode & PC_GIMBAL_MODE_AUTOAIM) != 0U) {
                gimbal_target.flags |= CONTROL_GIMBAL_FLAG_AUTOAIM;
            } else {
                gimbal_target.flags &=
                    (uint8_t)~CONTROL_GIMBAL_FLAG_AUTOAIM;
            }
        }
    }

    s_gimbal_target = gimbal_target;
    Comm_BoardLinkSetChassisCommand(&chassis_target);
}

static float Control_VisionYawStep(void)
{
    float error_deg = (float)visionDataGet.yaw_angle.yaw_predict * 0.01f;
    float raw_step = 0.0f;

    if (error_deg > CONTROL_VISION_YAW_DEADBAND_DEG ||
        error_deg < -CONTROL_VISION_YAW_DEADBAND_DEG) {
        raw_step = error_deg * CONTROL_VISION_YAW_GAIN;
    }
    s_vision_yaw_step +=
        CONTROL_VISION_YAW_FILTER_ALPHA * (raw_step - s_vision_yaw_step);
    return s_vision_yaw_step;
}

static void Control_GimbalDisable(void)
{
    GimbalYaw_SetMpcMode(0U, 0.0f);
    GimbalYaw_SetBigYawManualRate(0U, 0.0f);
    GimbalYaw_SetSmallYawRateDps(0.0f);
    GimbalPitch_SetGimbalPitchOutputState(0U);
    GimbalYaw_SetGimbalYawOutputState(0U);
    Shooter_ChangeShooterMode(Shoot_NULL);
    Shooter_ForceChangeFeederMode(Feeder_NULL);
    s_vision_yaw_step = 0.0f;
    s_gimbal_was_enabled = 0U;
}

static void Control_GimbalStep(void)
{
    GimbalPitch_GimbalPitchTypeDef *pitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *yaw = GimbalYaw_GetGimbalYawPtr();
    Protocol_DataTypeDef *protocol = Protocol_GetBusDataPtr();
    float yaw_step;
    float applied_yaw_step = 0.0f;
    float pitch_step;
    uint8_t autoaim_active;
    uint8_t tune_axis = 0U;
    uint8_t tune_locked = PC_Comm_IsGimbalTuneControlLocked();
    Control_YawBackendEnum yaw_backend = ControlYawBackend_Select(
        (uint8_t)((s_gimbal_target.flags & CONTROL_GIMBAL_FLAG_ENABLE) != 0U),
        (uint8_t)((s_gimbal_target.flags & CONTROL_GIMBAL_FLAG_MPC) != 0U),
        s_gimbal_target.legacy_yaw_pid, tune_locked,
        (uint8_t)(SMALL_YAW_ANGLE_OPEN_TEST != 0U));

    /* Left-switch DOWN selects the original cascaded PID/coordinator. The
     * former direct big-yaw rate mode is deliberately no longer requested. */
    GimbalYaw_SetBigYawManualRate(0U, 0.0f);

    if ((s_gimbal_target.flags & CONTROL_GIMBAL_FLAG_ENABLE) == 0U) {
        Control_GimbalDisable();
        return;
    }

    if (!s_gimbal_was_enabled) {
        yaw->yaw_ref = INS.YawTotalAngle;
        protocol->yaw_ref = INS.YawTotalAngle;
        pitch->pitch_ref = INS.Roll * PI / 180.0f +
                           Const_PITCH_MOTOR_INIT_OFFSETf;
        pitch->pitch_ref_smooth = pitch->pitch_ref;
        pitch->motor_ref = GimbalPitch_GetPositionFeedback();
        s_gimbal_was_enabled = 1U;
    }

    GimbalPitch_SetGimbalPitchOutputState(1U);
    GimbalYaw_SetGimbalYawOutputState(1U);

    yaw_step = (float)s_gimbal_target.yaw_rate_cdeg_s * 0.01f *
               ((float)CONTROL_TASK_PERIOD_MS / 1000.0f);
    pitch_step = (float)s_gimbal_target.pitch_rate_mrad_s * 0.001f *
                 ((float)CONTROL_TASK_PERIOD_MS / 1000.0f);

    autoaim_active = (uint8_t)(
        ((s_gimbal_target.flags & CONTROL_GIMBAL_FLAG_AUTOAIM) != 0U) &&
        AutoAim_IsOnline(CONTROL_VISION_TIMEOUT_MS));
    if (autoaim_active) {
        if (SMALL_YAW_ANGLE_OPEN_TEST == 0U)
            yaw_step += Control_VisionYawStep();
        else
            s_vision_yaw_step = 0.0f;
        pitch_step += (float)visionDataGet.pitch_angle.pitch_predict *
                      0.01f * (3.14159265358979323846f / 180.0f) *
                      CONTROL_VISION_PITCH_GAIN;
    } else {
        s_vision_yaw_step = 0.0f;
    }

    if (tune_locked) {
        tune_axis = PC_Comm_GetActiveTuneAxis();
    }

    if (tune_axis != PC_TUNE_AXIS_SMALL_YAW) {
        float step_cap = YawLimits_RateCap(GimbalYaw_TuneSmallManualMaxStepDeg);
        applied_yaw_step = Control_ClampF32(yaw_step, -step_cap, step_cap);
    }

    GimbalYaw_SetMpcMode((uint8_t)(yaw_backend == CONTROL_YAW_BACKEND_MPC),
        applied_yaw_step / ((float)CONTROL_TASK_PERIOD_MS / 1000.0f));

    if (tune_axis != PC_TUNE_AXIS_SMALL_YAW) {
        if (SMALL_YAW_ANGLE_OPEN_TEST != 0U)
            GimbalYaw_SetSmallYawRateDps(applied_yaw_step /
                ((float)CONTROL_TASK_PERIOD_MS / 1000.0f));
        else
            GimbalYaw_AddYawRef(applied_yaw_step);
    } else {
        GimbalYaw_SetSmallYawRateDps(0.0f);
    }
    protocol->yaw_ref = yaw->yaw_ref;
    if (tune_axis != PC_TUNE_AXIS_PITCH) {
        GimbalPitch_SetPitchRef(Gimbal_LimitPitch(pitch_step));
    }

    if ((s_gimbal_target.flags & CONTROL_GIMBAL_FLAG_SHOOTER) != 0U) {
        Shooter_ChangeShooterMode(Shoot_FAST);
        Shooter_ChangeFeederMode(
            (s_gimbal_target.flags & CONTROL_GIMBAL_FLAG_FIRE) != 0U
                ? Feeder_FAST_CONTINUE
                : Feeder_FINISH);
    } else {
        Shooter_ChangeShooterMode(Shoot_NULL);
        Shooter_ChangeFeederMode(Feeder_NULL);
    }
}
#endif

void Control_Init(void)
{
#if BOARD_HAS_GIMBAL
    memset(&s_gimbal_target, 0, sizeof(s_gimbal_target));
    Control_GimbalDisable();
#endif
#if BOARD_HAS_CHASSIS
    Control_ChassisStop();
#endif
}

void Control_Step(void)
{
#if BOARD_HAS_GIMBAL
    /* No blocking calls below: keep ownership stable through reference writes,
     * while allowing CAN, remote and IMU hardware interrupts to continue. */
    vTaskSuspendAll();
    if (YawIdentApp_OwnsControl()) {
        GimbalYaw_SetMpcMode(0U, 0.0f);
        GimbalYaw_SetBigYawManualRate(0U, 0.0f);
        Comm_ChassisCommandTypeDef stopped = {0};
        Control_StopChassisTarget(&stopped);
        Comm_BoardLinkSetChassisCommand(&stopped);
        Shooter_ChangeShooterMode(Shoot_NULL);
        Shooter_ForceChangeFeederMode(Feeder_NULL);
        s_gimbal_was_enabled = 0U;
        s_vision_yaw_step = 0.0f;
        (void)xTaskResumeAll();
        return; /* Gimbal_Task alone owns both references and output states. */
    }
    Control_UpdateTargets();
    Control_GimbalStep();
    (void)xTaskResumeAll();
#else
    Control_ChassisExecutorStep();
#endif
}

void Control_Task(void const *argument)
{
    uint32_t cycle = 0U;
    (void)DWT_GetDeltaT(&cycle);
    (void)argument;
    for (;;) {
        Control_DiagDtS = DWT_GetDeltaT(&cycle);
        Control_Step();
        osDelay(CONTROL_TASK_PERIOD_MS);
    }
}
