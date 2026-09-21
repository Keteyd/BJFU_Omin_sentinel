#ifndef MODULE_CHASSIS_H
#define MODULE_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "module_swerve.h"
#include <stdint.h>

typedef enum {
    Chassis_NULL = 0U,
    Chassis_SEP = 1U,
    Chassis_FOLLOW = 2U,
    Chassis_XTL = 3U,
    Chassis_GIMBAL = 4U
} Chassis_ModeEnum;

typedef Swerve_ModuleTypeDef Chassis_ChassisTypeDef;

typedef struct {
    Chassis_ModeEnum chassis_mode;
    /* Scaled command before the body-speed ramp. */
    float Chassis_TargetVx;
    float Chassis_TargetVy;
    float Chassis_TargetWz;
    /* Command applied to swerve kinematics after the body-speed ramp. */
    float Chassis_Vx;
    float Chassis_Vy;
    float Chassis_Wz;
    float Chassis_Yaw_Angle;
    float Chassis_Yaw_Rad;
    float Chassis_FontRight_AngleRef;
    float Chassis_FontLeft_AngleRef;
    float Chassis_BackLeft_AngleRef;
    float Chassis_BackRight_AngleRef;
    float Chassis_FontRight_SpeedRef;
    float Chassis_FontLeft_SpeedRef;
    float Chassis_BackLeft_SpeedRef;
    float Chassis_BackRight_SpeedRef;
} Chassis_StatusTypeDef;

/* Native chassis speed-reference units per second. Override at build time if needed. */
#ifndef CHASSIS_TRANSLATION_ACCEL_PER_S
#define CHASSIS_TRANSLATION_ACCEL_PER_S (600.0f)
#endif
#ifndef CHASSIS_TRANSLATION_DECEL_PER_S
#define CHASSIS_TRANSLATION_DECEL_PER_S (1200.0f)
#endif
#ifndef CHASSIS_TRANSLATION_STOP_PER_S
#define CHASSIS_TRANSLATION_STOP_PER_S (4000.0f)
#endif
#ifndef CHASSIS_YAW_ACCEL_PER_S
#define CHASSIS_YAW_ACCEL_PER_S (1200.0f)
#endif
#ifndef CHASSIS_YAW_DECEL_PER_S
#define CHASSIS_YAW_DECEL_PER_S (2400.0f)
#endif
#ifndef CHASSIS_YAW_STOP_PER_S
#define CHASSIS_YAW_STOP_PER_S (6000.0f)
#endif
#ifndef CHASSIS_RAMP_MAX_DT_S
#define CHASSIS_RAMP_MAX_DT_S (0.020f)
#endif
#ifndef CHASSIS_GIMBAL_TRANSLATION_DEADBAND_COUNTS
#define CHASSIS_GIMBAL_TRANSLATION_DEADBAND_COUNTS (15.0f)
#endif

extern Chassis_ChassisTypeDef Chassis_ControlData[4];
extern Chassis_StatusTypeDef Chassis_StatusData;
/* Raw steer encoder angles in order: back-right, back-left, front-right, front-left. */
extern float Chassis_SteerAngleDeg[4];

#ifndef CHASSIS_STEER0_SIGN
#define CHASSIS_STEER0_SIGN (1)
#endif
#ifndef CHASSIS_STEER1_SIGN
#define CHASSIS_STEER1_SIGN (1)
#endif
#ifndef CHASSIS_STEER2_SIGN
#define CHASSIS_STEER2_SIGN (1)
#endif
#ifndef CHASSIS_STEER3_SIGN
#define CHASSIS_STEER3_SIGN (1)
#endif

#ifndef CHASSIS_WHEEL0_SIGN
#define CHASSIS_WHEEL0_SIGN (-1)
#endif
#ifndef CHASSIS_WHEEL1_SIGN
#define CHASSIS_WHEEL1_SIGN (-1)
#endif
#ifndef CHASSIS_WHEEL2_SIGN
#define CHASSIS_WHEEL2_SIGN (-1)
#endif
#ifndef CHASSIS_WHEEL3_SIGN
#define CHASSIS_WHEEL3_SIGN (-1)
#endif

void Chassis_InitChassis(void);
Chassis_ChassisTypeDef *Chassis_ChassisPtr(void);
Chassis_StatusTypeDef *Chassis_StatusPtr(void);
void Chassis_SetChassisControlState(uint8_t state);
void Chassis_SetChassisOutputState(uint8_t state);
void Chassis_SetChassisMode(Chassis_ModeEnum mode);
void Chassis_SetChassisYawAngle(float yaw_angle, float yaw_angle_offset);
void Chassis_SetGimbalYaw(float yaw_deg);
void Chassis_SetChassisRef(float rc_vx, float rc_vy, float rc_wz);
void Chasssis_SetChasssisFontRightRef(float ref);
void Chassis_SetChassisFontLeftRef(float ref);
void Chassis_SetChassisBackLeftRef(float ref);
void Chassis_SetChassisBackRightRef(float ref);
void Chassis_Control(void);
void Chassis_Output(void);

#ifdef __cplusplus
}
#endif

#endif
