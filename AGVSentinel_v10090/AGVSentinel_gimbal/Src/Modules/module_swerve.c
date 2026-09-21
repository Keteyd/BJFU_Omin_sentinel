#include "module_swerve.h"

#include "alg_swerve_kinematics.h"
#include <string.h>

static void SwerveModule_InitPid(PID_PIDParamTypeDef *target,
                                 const float parameter[4][5])
{
    PID_InitPIDParam(target,
                     parameter[0][0], parameter[0][1], parameter[0][2],
                     parameter[0][3], parameter[0][4],
                     parameter[1][0], parameter[1][1],
                     parameter[2][0], parameter[2][1],
                     parameter[3][0], parameter[3][1], PID_POSITION);
}

void SwerveModule_Init(Swerve_ModuleTypeDef *module,
                       Actuator_MotorTypeDef *drive_motor,
                       Actuator_MotorTypeDef *steer_motor,
                       float steer_zero_deg,
                       float drive_sign,
                       float steer_sign,
                       const float drive_pid_param[4][5],
                       const float steer_pid_param[4][5])
{
    if (module == NULL || drive_pid_param == NULL || steer_pid_param == NULL) return;
    memset(module, 0, sizeof(*module));
    module->drive_motor = drive_motor;
    module->steer_motor = steer_motor;
    module->steer_zero_deg = steer_zero_deg;
    module->drive_sign = drive_sign;
    module->steer_sign = steer_sign;
    module->control_enabled = 1U;
    module->output_enabled = 1U;
    SwerveModule_InitPid(&module->drive_pid_param, drive_pid_param);
    SwerveModule_InitPid(&module->steer_pid_param, steer_pid_param);
    PID_ClearPID(&module->drive_pid);
    PID_ClearPID(&module->steer_pid);
    SwerveModule_Stop(module);
}

void SwerveModule_SetTarget(Swerve_ModuleTypeDef *module,
                            float angle_deg,
                            float speed)
{
    if (module == NULL) return;
    module->target_angle_deg = SwerveKinematics_Normalize360(angle_deg);
    module->target_speed = speed;
}

void SwerveModule_SetEnabled(Swerve_ModuleTypeDef *module,
                             uint8_t control_enabled,
                             uint8_t output_enabled)
{
    if (module == NULL) return;
    module->control_enabled = control_enabled != 0U;
    module->output_enabled = output_enabled != 0U;
    if (!module->output_enabled) SwerveModule_Stop(module);
}

void SwerveModule_Stop(Swerve_ModuleTypeDef *module)
{
    if (module == NULL) return;
    module->target_speed = 0.0f;
    (void)Actuator_MotorSetEffort(module->drive_motor, 0.0f);
    (void)Actuator_MotorSetEffort(module->steer_motor, 0.0f);
}

float SwerveModule_GetRawSteerAngle(Swerve_ModuleTypeDef *module)
{
    if (module == NULL) return 0.0f;
    module->raw_steer_angle_deg = Actuator_MotorGetFeedback(module->steer_motor).position;
    return module->raw_steer_angle_deg;
}

float SwerveModule_GetLogicalSteerAngle(Swerve_ModuleTypeDef *module)
{
    if (module == NULL) return 0.0f;
    module->logical_steer_angle_deg = SwerveKinematics_Normalize360(
        SwerveModule_GetRawSteerAngle(module) - module->steer_zero_deg);
    return module->logical_steer_angle_deg;
}

void SwerveModule_Control(Swerve_ModuleTypeDef *module)
{
    Actuator_FeedbackTypeDef drive_feedback;
    float angle_feedback;
    float angle_error;
    float virtual_feedback;

    if (module == NULL) return;
    if (!module->control_enabled || !module->output_enabled) {
        SwerveModule_Stop(module);
        return;
    }

    angle_feedback = SwerveModule_GetLogicalSteerAngle(module);
    angle_error = SwerveKinematics_WrapError(module->target_angle_deg - angle_feedback);
    virtual_feedback = module->target_angle_deg - angle_error;
    PID_SetPIDRef(&module->steer_pid, module->target_angle_deg);
    PID_SetPIDFdb(&module->steer_pid, virtual_feedback);
    PID_CalcPID(&module->steer_pid, &module->steer_pid_param);
    (void)Actuator_MotorSetEffort(
        module->steer_motor,
        PID_GetPIDOutput(&module->steer_pid) * module->steer_sign);

    drive_feedback = Actuator_MotorGetFeedback(module->drive_motor);
    PID_SetPIDRef(&module->drive_pid, module->target_speed);
    PID_SetPIDFdb(&module->drive_pid, drive_feedback.velocity * module->drive_sign);
    PID_CalcPID(&module->drive_pid, &module->drive_pid_param);
    (void)Actuator_MotorSetEffort(
        module->drive_motor,
        PID_GetPIDOutput(&module->drive_pid) * module->drive_sign);
}
