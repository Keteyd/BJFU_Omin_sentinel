#include "module_chassis.h"

#include "alg_swerve_kinematics.h"
#include "sys_const.h"
#include "sys_robot_actuators.h"
#include <math.h>
#include <string.h>

#define CHASSIS_MODULE_COUNT                 4U
#define CHASSIS_STEER_SMOOTH_FAST            (0.12f)
#define CHASSIS_STEER_SMOOTH_SLOW            (0.0008f)
#define CHASSIS_STEER_SMOOTH_SPEED_THRESHOLD (0.01f)
#define CHASSIS_STEER_MAX_STEP_DEG            (1.0f)
#define CHASSIS_GIMBAL_MAX_STEP_DEG           (5.0f)
#define CHASSIS_GIMBAL_FILTER_ALPHA           (0.05f)
#define CHASSIS_ZERO_SPEED_THRESHOLD          (0.01f)
#define CHASSIS_PI                            (3.14159265358979323846f)

Chassis_ChassisTypeDef Chassis_ControlData[CHASSIS_MODULE_COUNT];
Chassis_StatusTypeDef Chassis_StatusData;
float Chassis_SteerAngleDeg[CHASSIS_MODULE_COUNT];

static float s_previous_steer_ref[CHASSIS_MODULE_COUNT];
static float s_gimbal_yaw_deg;
static float s_filtered_gimbal_cos = 1.0f;
static float s_filtered_gimbal_sin;

static const RobotActuator_MotorIdEnum s_drive_ids[CHASSIS_MODULE_COUNT] = {
    ROBOT_MOTOR_CHASSIS_DRIVE_BR,
    ROBOT_MOTOR_CHASSIS_DRIVE_BL,
    ROBOT_MOTOR_CHASSIS_DRIVE_FR,
    ROBOT_MOTOR_CHASSIS_DRIVE_FL
};

static const RobotActuator_MotorIdEnum s_steer_ids[CHASSIS_MODULE_COUNT] = {
    ROBOT_MOTOR_CHASSIS_STEER_BR,
    ROBOT_MOTOR_CHASSIS_STEER_BL,
    ROBOT_MOTOR_CHASSIS_STEER_FR,
    ROBOT_MOTOR_CHASSIS_STEER_FL
};

static const float s_drive_signs[CHASSIS_MODULE_COUNT] = {
    CHASSIS_WHEEL0_SIGN, CHASSIS_WHEEL1_SIGN,
    CHASSIS_WHEEL2_SIGN, CHASSIS_WHEEL3_SIGN
};

static const float s_steer_signs[CHASSIS_MODULE_COUNT] = {
    CHASSIS_STEER0_SIGN, CHASSIS_STEER1_SIGN,
    CHASSIS_STEER2_SIGN, CHASSIS_STEER3_SIGN
};

static const float (*const s_drive_pid_params[CHASSIS_MODULE_COUNT])[5] = {
    Const_ChassisBackRightSpdParam,
    Const_ChassisBackLeftSpdParam,
    Const_ChassisFontRightSpdParam,
    Const_ChassisFontLeftSpdParam
};

static const float (*const s_steer_pid_params[CHASSIS_MODULE_COUNT])[5] = {
    Const_ChassisBackRightAngParam,
    Const_ChassisBackLeftAngParam,
    Const_ChassisFontRightAngParam,
    Const_ChassisFontLeftAngParam
};

static void Chassis_UpdateStatusTargets(
    const SwerveKinematics_ModuleStateTypeDef state[CHASSIS_MODULE_COUNT])
{
    Chassis_StatusData.Chassis_BackRight_AngleRef = state[0].angle_deg;
    Chassis_StatusData.Chassis_BackLeft_AngleRef = state[1].angle_deg;
    Chassis_StatusData.Chassis_FontRight_AngleRef = state[2].angle_deg;
    Chassis_StatusData.Chassis_FontLeft_AngleRef = state[3].angle_deg;
    Chassis_StatusData.Chassis_BackRight_SpeedRef = state[0].speed;
    Chassis_StatusData.Chassis_BackLeft_SpeedRef = state[1].speed;
    Chassis_StatusData.Chassis_FontRight_SpeedRef = state[2].speed;
    Chassis_StatusData.Chassis_FontLeft_SpeedRef = state[3].speed;
}

static void Chassis_ReadCurrentAngles(float logical_angle[CHASSIS_MODULE_COUNT])
{
    uint32_t i;
    for (i = 0U; i < CHASSIS_MODULE_COUNT; ++i) {
        Chassis_SteerAngleDeg[i] = SwerveModule_GetRawSteerAngle(&Chassis_ControlData[i]);
        logical_angle[i] = SwerveModule_GetLogicalSteerAngle(&Chassis_ControlData[i]);
    }
}

void Chassis_InitChassis(void)
{
    uint32_t i;
    memset(&Chassis_StatusData, 0, sizeof(Chassis_StatusData));
    memset(Chassis_SteerAngleDeg, 0, sizeof(Chassis_SteerAngleDeg));
    Chassis_StatusData.chassis_mode = Chassis_NULL;

    for (i = 0U; i < CHASSIS_MODULE_COUNT; ++i) {
        SwerveModule_Init(&Chassis_ControlData[i],
                          RobotActuators_GetMotor(s_drive_ids[i]),
                          RobotActuators_GetMotor(s_steer_ids[i]),
                          Const_ChassisSteerAngleOffset[i],
                          s_drive_signs[i], s_steer_signs[i],
                          s_drive_pid_params[i], s_steer_pid_params[i]);
        s_previous_steer_ref[i] = 0.0f;
    }
}

Chassis_ChassisTypeDef *Chassis_ChassisPtr(void)
{
    return Chassis_ControlData;
}

Chassis_StatusTypeDef *Chassis_StatusPtr(void)
{
    return &Chassis_StatusData;
}

void Chassis_SetChassisControlState(uint8_t state)
{
    uint32_t i;
    for (i = 0U; i < CHASSIS_MODULE_COUNT; ++i) {
        SwerveModule_SetEnabled(&Chassis_ControlData[i], state,
                                Chassis_ControlData[i].output_enabled);
    }
}

void Chassis_SetChassisOutputState(uint8_t state)
{
    uint32_t i;
    for (i = 0U; i < CHASSIS_MODULE_COUNT; ++i) {
        SwerveModule_SetEnabled(&Chassis_ControlData[i],
                                Chassis_ControlData[i].control_enabled, state);
    }
}

void Chassis_SetChassisMode(Chassis_ModeEnum mode)
{
    Chassis_StatusData.chassis_mode = mode;
}

void Chassis_SetChassisYawAngle(float yaw_angle, float yaw_angle_offset)
{
    Chassis_StatusData.Chassis_Yaw_Angle =
        SwerveKinematics_WrapError(yaw_angle - yaw_angle_offset);
    Chassis_StatusData.Chassis_Yaw_Rad =
        Chassis_StatusData.Chassis_Yaw_Angle * CHASSIS_PI / 180.0f;
}

void Chassis_SetGimbalYaw(float yaw_deg)
{
    s_gimbal_yaw_deg = yaw_deg;
}

void Chassis_SetChassisRef(float rc_vx, float rc_vy, float rc_wz)
{
    SwerveKinematics_ModuleStateTypeDef target[CHASSIS_MODULE_COUNT];
    float current_angle[CHASSIS_MODULE_COUNT];
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    float target_cos = 1.0f;
    float target_sin = 0.0f;
    float direction_length;
    float gimbal_cos = 1.0f;
    float gimbal_sin = 0.0f;
    uint32_t i;

    Chassis_ReadCurrentAngles(current_angle);
    switch (Chassis_StatusData.chassis_mode) {
        case Chassis_SEP:
            vx = rc_vx * REMOTE_CHASSIS_VX_GAIN;
            vy = rc_vy * REMOTE_CHASSIS_VY_GAIN;
            wz = rc_wz * REMOTE_CHASSIS_SEP_WZ_GAIN;
            break;
        case Chassis_GIMBAL:
            target_cos = cosf(s_gimbal_yaw_deg * CHASSIS_GIMBAL_YAW_STEER_SIGN * CHASSIS_PI / 180.0f);
            target_sin = sinf(s_gimbal_yaw_deg * CHASSIS_GIMBAL_YAW_STEER_SIGN * CHASSIS_PI / 180.0f);
            s_filtered_gimbal_cos += CHASSIS_GIMBAL_FILTER_ALPHA *
                                     (target_cos - s_filtered_gimbal_cos);
            s_filtered_gimbal_sin += CHASSIS_GIMBAL_FILTER_ALPHA *
                                     (target_sin - s_filtered_gimbal_sin);
            direction_length = sqrtf(s_filtered_gimbal_cos * s_filtered_gimbal_cos +
                                     s_filtered_gimbal_sin * s_filtered_gimbal_sin);
            gimbal_cos = s_filtered_gimbal_cos / (direction_length + 1.0e-6f);
            gimbal_sin = s_filtered_gimbal_sin / (direction_length + 1.0e-6f);
            vx = rc_vx * REMOTE_CHASSIS_VX_GAIN * gimbal_cos -
                 rc_vy * REMOTE_CHASSIS_VY_GAIN * gimbal_sin;
            vy = rc_vx * REMOTE_CHASSIS_VX_GAIN * gimbal_sin +
                 rc_vy * REMOTE_CHASSIS_VY_GAIN * gimbal_cos;
            wz = rc_wz * REMOTE_CHASSIS_SEP_WZ_GAIN;
            break;
        case Chassis_FOLLOW:
            vx = rc_vx * REMOTE_CHASSIS_VX_GAIN;
            vy = rc_vy * REMOTE_CHASSIS_VY_GAIN;
            wz = rc_wz * REMOTE_CHASSIS_FOLLOW_WZ_GAIN;
            if (wz > REMOTE_CHASSIS_FOLLOW_WZ_MAX) wz = REMOTE_CHASSIS_FOLLOW_WZ_MAX;
            if (wz < -REMOTE_CHASSIS_FOLLOW_WZ_MAX) wz = -REMOTE_CHASSIS_FOLLOW_WZ_MAX;
            break;
        case Chassis_XTL:
            vx = rc_vx * REMOTE_CHASSIS_VX_GAIN;
            vy = rc_vy * REMOTE_CHASSIS_VY_GAIN;
            wz = rc_wz * REMOTE_CHASSIS_FOLLOW_WZ_GAIN;
            break;
        case Chassis_NULL:
        default:
            for (i = 0U; i < CHASSIS_MODULE_COUNT; ++i) {
                target[i].angle_deg = current_angle[i];
                target[i].speed = 0.0f;
            }
            Chassis_UpdateStatusTargets(target);
            Chassis_StatusData.Chassis_Vx = 0.0f;
            Chassis_StatusData.Chassis_Vy = 0.0f;
            Chassis_StatusData.Chassis_Wz = 0.0f;
            return;
    }

    Chassis_StatusData.Chassis_Vx = vx;
    Chassis_StatusData.Chassis_Vy = vy;
    Chassis_StatusData.Chassis_Wz = wz;

    if (fabsf(vx) < CHASSIS_ZERO_SPEED_THRESHOLD &&
        fabsf(vy) < CHASSIS_ZERO_SPEED_THRESHOLD &&
        fabsf(wz) < CHASSIS_ZERO_SPEED_THRESHOLD) {
        if (Chassis_StatusData.chassis_mode == Chassis_GIMBAL) {
            SwerveKinematics_Solve(gimbal_cos, gimbal_sin, 0.0f,
                                   CHASSIS_SWERVE_L, CHASSIS_SWERVE_W,
                                   current_angle, target);
            for (i = 0U; i < CHASSIS_MODULE_COUNT; ++i) target[i].speed = 0.0f;
        } else {
            for (i = 0U; i < CHASSIS_MODULE_COUNT; ++i) {
                target[i].angle_deg = current_angle[i];
                target[i].speed = 0.0f;
            }
        }
    } else {
        SwerveKinematics_Solve(vx, vy, wz,
                               CHASSIS_SWERVE_L, CHASSIS_SWERVE_W,
                               current_angle, target);
    }
    Chassis_UpdateStatusTargets(target);
}

void Chassis_Control(void)
{
    float angle_ref[CHASSIS_MODULE_COUNT] = {
        Chassis_StatusData.Chassis_BackRight_AngleRef,
        Chassis_StatusData.Chassis_BackLeft_AngleRef,
        Chassis_StatusData.Chassis_FontRight_AngleRef,
        Chassis_StatusData.Chassis_FontLeft_AngleRef
    };
    const float speed_ref[CHASSIS_MODULE_COUNT] = {
        Chassis_StatusData.Chassis_BackRight_SpeedRef,
        Chassis_StatusData.Chassis_BackLeft_SpeedRef,
        Chassis_StatusData.Chassis_FontRight_SpeedRef,
        Chassis_StatusData.Chassis_FontLeft_SpeedRef
    };
    float speed_magnitude;
    float smoothing;
    float max_step;
    uint32_t i;

    if (Chassis_StatusData.chassis_mode == Chassis_NULL) {
        for (i = 0U; i < CHASSIS_MODULE_COUNT; ++i) SwerveModule_Stop(&Chassis_ControlData[i]);
        return;
    }

    speed_magnitude = sqrtf(Chassis_StatusData.Chassis_Vx * Chassis_StatusData.Chassis_Vx +
                            Chassis_StatusData.Chassis_Vy * Chassis_StatusData.Chassis_Vy) +
                      fabsf(Chassis_StatusData.Chassis_Wz) *
                      sqrtf(CHASSIS_SWERVE_L * CHASSIS_SWERVE_L +
                            CHASSIS_SWERVE_W * CHASSIS_SWERVE_W);
    smoothing = speed_magnitude < CHASSIS_STEER_SMOOTH_SPEED_THRESHOLD
                    ? CHASSIS_STEER_SMOOTH_SLOW
                    : CHASSIS_STEER_SMOOTH_FAST;
    max_step = CHASSIS_STEER_MAX_STEP_DEG;
    if (Chassis_StatusData.chassis_mode == Chassis_GIMBAL) {
        smoothing = CHASSIS_STEER_SMOOTH_FAST;
        max_step = CHASSIS_GIMBAL_MAX_STEP_DEG;
    }

    for (i = 0U; i < CHASSIS_MODULE_COUNT; ++i) {
        float delta = smoothing *
                      SwerveKinematics_WrapError(angle_ref[i] - s_previous_steer_ref[i]);
        if (delta > max_step) delta = max_step;
        if (delta < -max_step) delta = -max_step;
        s_previous_steer_ref[i] = SwerveKinematics_Normalize360(
            s_previous_steer_ref[i] + delta);
        SwerveModule_SetTarget(&Chassis_ControlData[i],
                               s_previous_steer_ref[i], speed_ref[i]);
        SwerveModule_Control(&Chassis_ControlData[i]);
    }
}

void Chassis_Output(void)
{
    uint32_t i;
    for (i = 0U; i < CHASSIS_MODULE_COUNT; ++i) {
        if (!Chassis_ControlData[i].output_enabled) SwerveModule_Stop(&Chassis_ControlData[i]);
    }
    (void)Actuator_GroupFlush(RobotActuators_GetGroup(ROBOT_GROUP_CHASSIS_DRIVE));
    (void)Actuator_GroupFlush(RobotActuators_GetGroup(ROBOT_GROUP_CHASSIS_STEER));
}

void Chasssis_SetChasssisFontRightRef(float ref) { (void)ref; }
void Chassis_SetChassisFontLeftRef(float ref) { (void)ref; }
void Chassis_SetChassisBackLeftRef(float ref) { (void)ref; }
void Chassis_SetChassisBackRightRef(float ref) { (void)ref; }
