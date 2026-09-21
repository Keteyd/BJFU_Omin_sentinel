/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_const.c
 *  Description  : This file include all required constants
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:20:29
 *  LastEditTime : 2023-05-07 10:55:49
 */


#include "sys_const.h"
#include "module_yaw_limits.h"
#include "module_big_yaw_tune.h"
#include <float.h>

float Const_SERVO_INIT_OFFSET = 0.053f;

const float Const_Shooter15mpers        = 198.0f;
const float Const_Shooter18mpers        = 220.0f;
const float Const_Shooter22mpers        = 260.0f;
const float Const_Shooter30mpers        = 378.0f;

const float Const_ShooterLockedCurrent              = 10000.0f;//堵转电流阈值
const float Const_ShooterLockedSpeed                = 10.0f;//堵转速度阈值
const float Const_ShooterLockedTime                 = 7.0f;//连续判定次数阈值
const float Const_ShooterRelockedTime               = 150.0f;
const float Const_ShooterLockedReverseSpeed         = -150.0f;

const float Const_FeederSlowSpeed                   = 60.0f;
const float Const_FeederFastSpeed                   = 80.0f;
const float Const_FeederWaitSpeed                   = 0.0f;

const float Const_HeatCtrlFastLimit                 = 75;
const float Const_HeatCtrlSlowLimit                 = 40;
const float Const_HeatCtrlWaitLimit                 = 10;
const float Const_HeatCtrlSingleCount               = 10;
const float Const_HeatCtrlStopLimit                 = 10;

const float Const_ShooterSlowSpeed                  = 150.0f;
const float Const_ShooterFastSpeed                  = 330.0f;

const float MOUSE_PITCH_ANGLE_TO_FACT             = 0.008f;
	  float MOUSE_YAW_ANGLE_TO_FACT               = 0.0025f;
const float MOUSE_CHASSIS_TO_FACT                 = 0.00022;       
const float MOUSE_CHASSIS_ACCELERATE              = 0.02f;
	  float MOUSE_CHASSIS_MAX_SPEED               	  = 12;
const float MOUSE_LEG_LEN_DLEN                    = 0.00018;

const float Const_WHEELLEG_REMOTE_YAW_GAIN              = 0.0002;//0.0005
const float Const_WHEELLEG_REMOTE_X_GAIN                = 0.000022;
const float Const_WHEELLEG_REMOTE_LEN_GAIN              = 0.00000028;
const float Const_Vision_YAW_GAIN                       = 0.0005;
const float Const_Vision_PITCH_GAIN                     =0.01;

const float REMOTE_PITCH_ANGLE_TO_REF                   = 0.0005f;//0.0005

const float REMOTE_CHASSIS_VX_GAIN                  = 0.5f;
const float REMOTE_CHASSIS_VY_GAIN                  = 0.5f;  
const float REMOTE_CHASSIS_SEP_WZ_GAIN							= -0.25f;
const float REMOTE_CHASSIS_FOLLOW_WZ_GAIN						= 1.0f;
const float REMOTE_CHASSIS_FOLLOW_WZ_MAX						= 175.0f;
const float REMOTE_CHASSIS_XTL_WZ_GAIN							= 1.0f;
const float CHASSIS_YAW_ANGLE_OFFSET							= 125.0f;	//底盘坐标系对齐用的 offset,当车头正对定义的“正前方”时，IMU/云台给出的原始 yaw 角是多少度
float CHASSIS_GIMBAL_YAW_STEER_OFFSET							= -57.0f; 	// 遥控器左中右上Gimbal模式下额外校准偏置(deg)，用于修正“云台yaw-舵向角”方向不一致（允许调试器动态修改）
float CHASSIS_GIMBAL_YAW_STEER_SIGN								= -1.0f; 	// 跟随方向系数：1 正常；-1 镜像反向（允许调试器动态修改）
const float CHASSIS_XTL_WZ													= 0.0f;//小陀螺转速,导航下发的话就可以给0了,以免pc断开串口wz又被小陀螺模式常量接管伤人
const float CHASSIS_SWERVE_L												= 0.282f;  /* 舵轮半长 m，按车体改 */
const float CHASSIS_SWERVE_W												= 0.282f;  /* 舵轮半宽 m，按车体改 */
/* 舵向零位(deg)：轮子物理朝前时该轮 limited_angle 的读数，顺序 [右后, 左后, 右前, 左前]
 * 校准：上电后不动摇杆，用手把四轮都掰到“车体正前方”，用调试读四路 limited_angle 填入下面 */
const float Const_ChassisSteerAngleOffset[4] = {
    135.83f,  /* [0] 右后 BackRight   */
    285.0f,  /* [1] 左后 BackLeft    */
    101.76f,  /* [2] 右前 FontRight   */
    197.08f   /* [3] 左前 FontLeft    */
};
const float Const_Chasiss_Navigate_angular_z        =600.0f;
const float Const_Chasiss_Navigate_linear_x         =300.f;
const float Const_Chasiss_Navigate_linear_y         =300.f;
const float Const_YAW_AUTOADD														= 200.0f;//200
const float Const_PITCH_AUTOADD													= 5.0f;
int   Const_ADD_MAX																= 2000;

/* DM4310 multi-turn positions measured at the physical Pitch stops. */
float Const_PITCH_UMAXANGLE                       = 0.259f;
float Const_PITCH_UMAXANGLE_GRYO                  = 0.259f;
float Const_PITCH_DMAXANGLE                       = -0.455f;
float Const_YAW_MAXANGLE                          = 40.0f; //在普通模式下把 yaw 可偏转范围限制在一个最大角度
float Const_PITCH_MOTOR_INIT_OFFSETf              = 0.0f;

/* 云台大 yaw 轴 6020 零位(deg)：物理朝前时该电机 limited_angle 的读数。
 * 校准：上电后 yaw 不转，用手把云台 yaw 掰到“正前方”，用调试读 Motor_Big_YawMotor.encoder.limited_angle 填入下面 */
const float Const_GimbalBigYawAngleOffset          = 0.0f;
/* 云台小 yaw 零位(deg)：锁死目标 = (limited_angle - offset) == 0 */
const float Const_GimbalSmallYawAngleOffset        = YAW_LIMIT_ZERO_DEG;

/*
 * Encoder big-yaw hold/relief is enabled below SAFE. The effort
 * value is multiplied by 1000 before it is sent to the DJI motor. Change these
 * while the remote is in SAFE; each newly enabled axis captures its position.
 */
volatile uint8_t GimbalYaw_DiagBigEnable           = !BIG_YAW_OUTPUT_INHIBIT_TEST;
volatile uint8_t GimbalYaw_DiagSmallEnable         = !BIG_YAW_PASSIVE_SMALL_TEST;
volatile float GimbalYaw_DiagBigDirection          = 1.0f;
/* Positive big encoder motion is expected to relieve positive small yaw. */
volatile float GimbalYaw_BigReliefDirection        = 1.0f;
volatile float GimbalYaw_DiagSmallDirection        = 1.0f;
volatile float GimbalYaw_DiagEffortLimit           = 6.0f;

/* Pitch IMU outer loop. DM4310 keeps its own MIT position/velocity loop. */
const float Const_GimbalPitchImuKp                 = 14.0f;
const float Const_GimbalPitchImuKd                 = 0.30f;
const float Const_GimbalPitchImuRateMaxRadS        = 2.0f;
const float Const_GimbalPitchImuAngleDeadbandRad   = 0.0026f; /* 0.15 deg */
const float Const_GimbalPitchImuRateDeadbandRadS   = 0.020f;
const float Const_GimbalPitchTargetFilterAlpha     = 0.20f;
volatile uint8_t GimbalPitch_ImuEnable             = 1U;
volatile float GimbalPitch_ImuDirection            = 1.0f;

// DM4310 (Pitch) on CAN2
const uint16_t Const_DM4310_PitchMotorId           = 0x01;
const uint16_t Const_DM4310_PitchFeedbackId        = 0x00;
const uint32_t Const_DM4310_OfflineMs              = 200;

// pitch gimbal param
const float Const_GimbalPitchSpdParam[4][5] = {
    {5.0f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_GimbalPitchAngParam[4][5] = {
    {0.5f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
/* Big-yaw disturbance-rejection commissioning defaults, independent of small yaw. */
const float Const_GimbalYawSpdParam[4][5] = {
    {BIG_YAW_SPEED_KP, BIG_YAW_SPEED_KI, BIG_YAW_SPEED_KD, 0.0f, BIG_YAW_EFFORT_DEFAULT}, {0.1f, -1}, {0, 0}, {-1, -1}};

/* No fixed rpm cap; position error and motor effort remain bounded. */
const float Const_GimbalYawAngParam[4][5] = {
    {BIG_YAW_ANGLE_KP, BIG_YAW_ANGLE_KI, BIG_YAW_ANGLE_KD, 0.0f, FLT_MAX}, {0.1f, -1}, {0, 0}, {-1, -1}};

/* Small yaw uses IMU yaw (deg) -> IMU yaw rate (rpm) -> motor effort. */
const float Const_GimbalSmallYawSpdParam[4][5] = {
    {0.60f, 0.0f, 0.0f, 0.0f, 6.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_GimbalSmallYawAngParam[4][5] = {
    {2.60f, 0.0f, 0.0f, 0.0f, FLT_MAX}, {0.1f, -1}, {0, 0}, {-1, -1}};
//const float Const_GimbalYawSpdParam[4][5] = {
//    {2.5f, 0.0, 0.0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

//const float Const_GimbalYawAngParam[4][5] = {
//    {0.45f, 0.01, 0.35, 0, 10.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

// shooter param
const float Const_ShootLeftParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};//kp,ki,kd,sum_max,_output_max
const float Const_ShootRightParam[4][5] = {
    {0.04f, 0.01f, 0.04, 40, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_FeedAngParam[4][5] = {
    {0.6f, 0, 0, 0, 20.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_FeedSpdParam[4][5] = {
    {0.1f, 0, 0, 0, 10.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

// Chassis param（舵向角环 P 降低、加 D，减轻抽搐）
const float Const_ChassisFontRightAngParam[4][5] = {
    {0.4f, 0.0f, 0.0f, 0.0f, 15.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisFontRightSpdParam[4][5] = {
    {0.03f, 0.01f, 0.03f, 40, 15.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisFontLeftAngParam[4][5] = {
    {0.4f, 0.0f, 0.0f, 0.0f, 15.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisFontLeftSpdParam[4][5] = {
    {0.03f, 0.01f, 0.03f, 40, 15.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackLeftAngParam[4][5] = {
    {0.4f, 0.0f, 0.0f, 0.0f, 15.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackLeftSpdParam[4][5] = {
    {0.03f, 0.01f, 0.03f, 40, 15.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackRightAngParam[4][5] = {
    {0.4f, 0.0f, 0.0f, 0.0f, 15.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};
const float Const_ChassisBackRightSpdParam[4][5] = {
    {0.03f, 0.01f, 0.03f, 40, 15.0f}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                   0, 1, 0, 0, 0, 0,
                                   0, 0, 1, 0, 0, 0,
                                   0, 0, 0, 1, 0, 0,
                                   0, 0, 0, 0, 1, 0,
                                   0, 0, 0, 0, 0, 1};

float QuaternionEKF_P[36] = {100000,    0.1,    0.1,    0.1,    0.1,    0.1,
                                0.1, 100000,    0.1,    0.1,    0.1,    0.1,
                                0.1,    0.1, 100000,    0.1,    0.1,    0.1,
                                0.1,    0.1,    0.1, 100000,    0.1,    0.1,
                                0.1,    0.1,    0.1,    0.1,    100,    0.1,
                                0.1,    0.1,    0.1,    0.1,    0.1,    100};
