#ifndef IDENT_FIXTURE_H
#define IDENT_FIXTURE_H
#include <stdint.h>
#include <string.h>
#include <math.h>
#define SMALL_YAW_ANGLE_OPEN_TEST 0U
#define BIG_YAW_PASSIVE_SMALL_TEST 0U
#define Remote_STATE_CONNECTED 1U
#define Remote_SWITCH_UP 1U
#define Remote_SWITCH_DOWN 2U
#define Remote_SWITCH_MIDDLE 3U
#define Motor1 0
#define PI 3.14159265358979323846f
#define Const_PITCH_MOTOR_INIT_OFFSETf 0.0f
typedef struct {
    float yaw_deg, roll_deg, pitch_deg, gyro[3], dt_s;
    uint32_t sequence, tick_ms;
} INS_Observation;
typedef struct {
    struct { int16_t ch[5]; uint8_t s[2]; } remote;
    struct { int16_t x, y; uint8_t l, r; } mouse;
    uint32_t last_update_time;
    uint8_t state;
} Remote_RemoteDataTypeDef;
typedef struct {
    struct { float limited_angle, speed; uint16_t angle; } encoder;
    float output; uint32_t last_update_time; uint8_t is_online;
} Motor_MotorTypeDef;
typedef struct { float pitch_ref, pitch_ref_smooth, motor_ref; uint8_t output_state, control_state; } GimbalPitch_GimbalPitchTypeDef;
typedef struct { float yaw_ref; uint8_t output_state, control_state, yaw_ref_limit_status; } GimbalYaw_GimbalYawTypeDef;
typedef struct { struct { float kp_set, kd_set; } ctrl; } MockDm;
extern MockDm motor[1];
extern Motor_MotorTypeDef Motor_Big_YawMotor, Motor_Small_YawMotor;
extern uint8_t GimbalYaw_DiagSmallEnable, GimbalYaw_DiagBigActive, GimbalYaw_DiagBigSaturated,
    GimbalYaw_DiagBigStopReason, GimbalPitch_ImuEnable;
extern uint8_t GimbalPitch_DiagMotorOnline;
extern float GimbalYaw_TuneBigAngKp;
extern float GimbalYaw_TuneBigSpdKp;
extern float GimbalYaw_TuneBigEffortLimit;
extern float GimbalYaw_TuneBigSpeedFilterTauS;
extern float GimbalYaw_TuneBigAngKi;
extern float GimbalYaw_TuneBigAngKd;
extern float GimbalYaw_TuneBigSpdKi;
extern float GimbalYaw_TuneBigSpdKd;
extern float GimbalYaw_TuneSmallAngKp;
extern float GimbalYaw_TuneSmallSpdKp;
extern float GimbalYaw_DiagEffortLimit;
extern float GimbalYaw_TuneSmallSpeedFilterAlpha;
extern float GimbalYaw_TuneSmallAngKi;
extern float GimbalYaw_TuneSmallAngKd;
extern float GimbalYaw_TuneSmallSpdKi;
extern float GimbalYaw_TuneSmallSpdKd;
extern float GimbalYaw_TuneSmallSpeedLimitRpm;
extern float GimbalYaw_TuneSmallManualMaxStepDeg;
extern float GimbalYaw_DiagSmallDirection;
extern float GimbalYaw_DiagBigDirection;
extern float GimbalYaw_DiagBigSpeedRefRpm;
extern float GimbalYaw_DiagSmallSpeedRefRpm;
extern float GimbalPitch_TuneImuKp;
extern float GimbalPitch_TuneImuKd;
extern float GimbalPitch_TuneGravityEffort;
extern float GimbalPitch_TuneRateLimitRadS;
extern float GimbalPitch_TuneAngleDeadbandRad;
extern float GimbalPitch_TuneRateDeadbandRadS;
extern float GimbalYaw_TuneSmallStepDeg;
extern float GimbalPitch_TuneStepRad;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void __DMB(void);
void INS_ReadObservation(INS_Observation *o);
uint8_t INS_IsReady(void);
Remote_RemoteDataTypeDef *Remote_GetRemoteDataPtr(void);
GimbalYaw_GimbalYawTypeDef *GimbalYaw_GetGimbalYawPtr(void);
GimbalPitch_GimbalPitchTypeDef *GimbalPitch_GetGimbalPitchPtr(void);
void GimbalYaw_SetGimbalYawOutputState(uint8_t state);
void GimbalPitch_SetGimbalPitchOutputState(uint8_t state);
void GimbalYaw_SetSmallYawRateDps(float rate);
float GimbalPitch_GetPositionFeedback(void);
void RobotActuators_GetPitchMitGains(float *kp, float *kd);
void GimbalYaw_Output(void);
void GimbalPitch_Output(void);
uint8_t PC_Comm_IsRemoteSafe(void);
uint8_t PC_Comm_IsGimbalTuneSessionOnline(void);
uint8_t PC_Comm_SendPacket(uint8_t cmd, const void *data, uint8_t size);
uint8_t PC_Comm_SendBatch(uint8_t cmd, const uint8_t *data, uint8_t count);
uint8_t PC_Comm_SendTrace(uint32_t id, uint16_t sequence, const void *record);
#endif
