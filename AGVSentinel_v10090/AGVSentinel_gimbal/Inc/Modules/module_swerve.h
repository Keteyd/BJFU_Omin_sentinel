#ifndef MODULE_SWERVE_H
#define MODULE_SWERVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "alg_pid.h"
#include "lib_actuator.h"
#include <stdint.h>

typedef struct {
    Actuator_MotorTypeDef *drive_motor;
    Actuator_MotorTypeDef *steer_motor;
    float steer_zero_deg;
    float drive_sign;
    float steer_sign;
    float target_angle_deg;
    float target_speed;
    float raw_steer_angle_deg;
    float logical_steer_angle_deg;
    uint8_t control_enabled;
    uint8_t output_enabled;
    PID_PIDTypeDef drive_pid;
    PID_PIDParamTypeDef drive_pid_param;
    PID_PIDTypeDef steer_pid;
    PID_PIDParamTypeDef steer_pid_param;
} Swerve_ModuleTypeDef;

void SwerveModule_Init(Swerve_ModuleTypeDef *module,
                       Actuator_MotorTypeDef *drive_motor,
                       Actuator_MotorTypeDef *steer_motor,
                       float steer_zero_deg,
                       float drive_sign,
                       float steer_sign,
                       const float drive_pid_param[4][5],
                       const float steer_pid_param[4][5]);
void SwerveModule_SetTarget(Swerve_ModuleTypeDef *module,
                            float angle_deg,
                            float speed);
void SwerveModule_SetEnabled(Swerve_ModuleTypeDef *module,
                             uint8_t control_enabled,
                             uint8_t output_enabled);
void SwerveModule_Stop(Swerve_ModuleTypeDef *module);
void SwerveModule_Control(Swerve_ModuleTypeDef *module);
float SwerveModule_GetRawSteerAngle(Swerve_ModuleTypeDef *module);
float SwerveModule_GetLogicalSteerAngle(Swerve_ModuleTypeDef *module);

#ifdef __cplusplus
}
#endif

#endif
