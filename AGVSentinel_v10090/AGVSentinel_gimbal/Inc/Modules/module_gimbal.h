/*
 *  Project      : Polaris
 * 
 *  file         : cha_gimbal_ctrl.h
 *  Description  : This file contains Gimbal control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 11:15:17
 */

#ifndef MODULE_GIMBAL_H
#define MODULE_GIMBAL_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "alg_math.h"
#include "alg_pid.h"
#include "module_small_yaw_rate.h"

typedef struct {
    float pitch_ref;                                  // 云台Pitch轴角度目标值（未滤波）
    float pitch_ref_smooth;                           // 低通滤波后的Pitch目标值
    float filter_alpha;                               // Pitch目标值一阶低通滤波系数
    float motor_ref;                                  // DM4310 position target, rad
    float imu_position_fdb;                           // IMU roll feedback, rad
    float imu_speed_fdb;                              // IMU roll rate feedback, rad/s
    float imu_error;                                  // Deadbanded attitude error, rad
    float motor_rate_ref;                             // Integrated motor target rate, rad/s
    float motor_zero_ref;                             // Startup motor position, rad
    float motor_min_ref;                              // Runtime lower mechanical guard, rad
    float motor_max_ref;                              // Runtime upper mechanical guard, rad
    float imu_zero_ref;                               // Startup world pitch, rad
    float imu_min_ref;                                // Runtime lower world target guard, rad
    float imu_max_ref;                                // Runtime upper world target guard, rad
    uint8_t limits_initialized;
    uint8_t pitch_ref_limit_status;                   // Gimbal Pitch limit status 
    uint8_t pitch_count;

    uint8_t control_state;                          // Whether to enable control 1 Yes 0 No 
    uint8_t output_state;                           // Whether to enable output 1 Yes 0 No 
    uint8_t pending_state;                          // Gimbal Pitch Occupy Lock 1 Yes 0 No 

    PID_PIDTypeDef spdPID;
    PID_PIDParamTypeDef spdPIDParam;
    PID_PIDTypeDef angPID;
    PID_PIDParamTypeDef angPIDParam;

} GimbalPitch_GimbalPitchTypeDef;

typedef struct {
    float yaw_ref;                                  // Gimbal Yaw angle target value 
    float yaw_position_fdb;                         // Gimbal Yaw IMU angle feedback value 
    float yaw_speed_fdb;                            // Gimbal Yaw IMU angular velocity feedback value
    uint8_t yaw_ref_limit_status;                   // Gimbal Yaw limit status 
    uint8_t yaw_count;

    uint8_t control_state;                          // Whether to enable control 1 Yes 0 No 
    uint8_t output_state;                           // Whether to enable output 1 Yes 0 No 
    uint8_t pending_state;                          // Gimbal Yaw Occupy Lock 1 Yes 0 No 

    PID_PIDTypeDef spdPID;
    PID_PIDParamTypeDef spdPIDParam;
    PID_PIDTypeDef angPID;
    PID_PIDParamTypeDef angPIDParam;
	
	PID_PIDTypeDef GimbalSmallYaw_spdPID;
	PID_PIDParamTypeDef GimbalSmallYaw_spdPIDParam;
	PID_PIDTypeDef GimbalSmallYaw_angPID;
	PID_PIDParamTypeDef GimbalSmallYaw_angPIDParam;
	
	

} GimbalYaw_GimbalYawTypeDef;



extern GimbalPitch_GimbalPitchTypeDef GimbalPitch_gimbalPitchControlData;
extern GimbalYaw_GimbalYawTypeDef GimbalYaw_gimbalYawControlData;

/* Commissioning telemetry. Effort values are scaled by 1000 for the CAN command. */
extern volatile float GimbalYaw_DiagBigEffortCmd;
extern volatile float GimbalYaw_DiagBigErrorDeg;
extern volatile float GimbalYaw_DiagBigSpeedRefRpm;
extern volatile float GimbalYaw_DiagBigSpeedRpm;
extern volatile float GimbalYaw_DiagBigCurrent;
extern volatile float GimbalYaw_DiagBigTemperatureC;
extern volatile uint8_t GimbalYaw_DiagBigSaturated;
extern volatile uint8_t GimbalYaw_DiagBigMode;
extern volatile uint8_t GimbalYaw_DiagBigActive;
typedef enum {
    GIMBAL_BIG_STOP_NONE = 0,
    GIMBAL_BIG_STOP_ALLOW,
    GIMBAL_BIG_STOP_TUNE,
    GIMBAL_BIG_STOP_EFFORT_LIMIT,
    GIMBAL_BIG_STOP_FEEDBACK,
    GIMBAL_BIG_STOP_SMALL_INTERLOCK,
    GIMBAL_BIG_STOP_SMALL_LIMITS,
    GIMBAL_BIG_STOP_DIRECTION,
    GIMBAL_BIG_STOP_COORDINATOR,
    GIMBAL_BIG_STOP_ANGLE_PID,
    GIMBAL_BIG_STOP_SPEED_REFERENCE,
    GIMBAL_BIG_STOP_SPEED_PID,
    GIMBAL_BIG_STOP_EFFORT_OUTPUT
} GimbalYaw_BigStopReason;
extern volatile uint8_t GimbalYaw_DiagBigStopReason;
extern volatile float GimbalYaw_DiagSmallEffortCmd;
extern volatile float GimbalYaw_DiagSmallRefDeg;
extern volatile float GimbalYaw_DiagSmallFdbDeg;
extern volatile float GimbalYaw_DiagSmallRawSpeedRpm;
extern volatile float GimbalYaw_DiagSmallSpeedRpm;
extern volatile float GimbalYaw_DiagSmallErrorDeg;
extern volatile float GimbalYaw_DiagSmallSpeedRefRpm;
extern volatile float GimbalYaw_DiagSmallCurrent;
extern volatile float GimbalYaw_DiagSmallJointDeg;
extern volatile uint8_t GimbalYaw_DiagSmallLimitsValid;
/* Deployed speed-reference MPC diagnostics; references are deg/s. */
extern volatile uint8_t GimbalYaw_DiagMpcRequested;
extern volatile uint8_t GimbalYaw_DiagMpcActive;
extern volatile uint8_t GimbalYaw_DiagMpcUpdated;
extern volatile uint8_t GimbalYaw_DiagMpcReason;
extern volatile float GimbalYaw_DiagMpcBigRefDps;
extern volatile float GimbalYaw_DiagMpcSmallRefDps;
extern volatile float GimbalYaw_DiagMpcBigUnconstrainedDps;
extern volatile float GimbalYaw_DiagMpcSmallUnconstrainedDps;

/* Big-yaw live tuning is independent of the small-yaw serial tuning packet. */
extern volatile float GimbalYaw_TuneBigAngKp;
extern volatile float GimbalYaw_TuneBigSpdKp;
extern volatile float GimbalYaw_TuneBigAngKi;
extern volatile float GimbalYaw_TuneBigAngKd;
extern volatile float GimbalYaw_TuneBigSpdKi;
extern volatile float GimbalYaw_TuneBigSpdKd;
extern volatile float GimbalYaw_TuneBigEffortLimit;
extern volatile float GimbalYaw_TuneBigSpeedFilterTauS;

/* Small-yaw live tuning variables. They may be edited from the Keil Watch window. */
extern volatile float GimbalYaw_TuneSmallAngKp;
extern volatile float GimbalYaw_TuneSmallAngKi;
extern volatile float GimbalYaw_TuneSmallAngKd;
extern volatile float GimbalYaw_TuneSmallSpdKp;
extern volatile float GimbalYaw_TuneSmallSpdKi;
extern volatile float GimbalYaw_TuneSmallSpdKd;
extern volatile float GimbalYaw_TuneSmallSpeedLimitRpm;
extern volatile float GimbalYaw_TuneSmallSpeedFilterAlpha;
extern volatile float GimbalYaw_TuneSmallManualMaxStepDeg;
extern volatile float GimbalYaw_TuneSmallStepDeg;
extern volatile uint8_t GimbalYaw_TuneResetRequest;

/* Pitch IMU outer-loop tuning and telemetry. Gravity effort is signed. */
extern volatile float GimbalPitch_TuneImuKp;
extern volatile float GimbalPitch_TuneImuKd;
extern volatile float GimbalPitch_TuneRateLimitRadS;
extern volatile float GimbalPitch_TuneGravityEffort;
extern volatile float GimbalPitch_TuneAngleDeadbandRad;
extern volatile float GimbalPitch_TuneRateDeadbandRadS;
extern volatile float GimbalPitch_TuneStepRad;
extern volatile float GimbalPitch_DiagAppliedStepRad;
extern volatile uint8_t GimbalPitch_TuneResetRequest;
extern volatile float GimbalPitch_DiagGravityEffort;
extern volatile float GimbalPitch_DiagMotorPosition;
extern volatile float GimbalPitch_DiagMotorSpeed;
extern volatile float GimbalPitch_DiagMotorEffort;
extern volatile uint8_t GimbalPitch_DiagMotorOnline;


void GimbalPitch_InitGimbalPitch(void);
GimbalPitch_GimbalPitchTypeDef* GimbalPitch_GetGimbalPitchPtr(void);
void GimbalPitch_SetGimbalPitchControlState(uint8_t state);
void GimbalPitch_SetGimbalPitchOutputState(uint8_t state);
void GimbalPitch_SetPitchRef(float pitch_ref);
float Gimbal_LimitPitch(float ref);
float Gimbal_LimitYaw(float ref);
void GimbalPitch_Control(void);
void GimbalPitch_Output(void);
float GimbalPitch_GetPositionFeedback(void);


void GimbalYaw_InitGimbalYaw(void);
GimbalYaw_GimbalYawTypeDef* GimbalYaw_GetGimbalYawPtr(void);
void GimbalYaw_SetGimbalYawControlState(uint8_t state);
void GimbalYaw_SetGimbalYawOutputState(uint8_t state);
void GimbalYaw_SetYawRef(float yaw_ref);
void GimbalYaw_AddYawRef(float yaw_step_deg);
void GimbalYaw_AddSmallYawRef(float yaw_step_deg);
void GimbalYaw_SetSmallYawRateDps(float rate_dps);
void GimbalYaw_SetBigYawManualRate(uint8_t enabled, float rate_dps);
void GimbalYaw_SetMpcMode(uint8_t enabled, float target_rate_dps);
extern volatile uint8_t GimbalYaw_DiagBigManualFault;
void GimbalYaw_SetIMUYawPositionFdb(float imu_yaw_position_fdb);
void GimbalYaw_SetIMUYawSpeedFdb(float imu_yaw_speed_fdb);
void GimbalYaw_Control(void);
void GimbalYaw_Output(void);
/** 校准：读大 yaw 编码器原始角(0~360)，朝前时填入 Const_GimbalBigYawAngleOffset */
float GimbalYaw_GetEncoderRawDeg(void);
/** 校准：逻辑角 = 编码器 - offset，填好 offset 后朝前为 0° */
float GimbalYaw_GetEncoderLogicalDeg(void);

#endif

#ifdef __cplusplus
}
#endif
