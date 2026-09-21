/*
 *  Project      : Polaris
 * 
 *  file         : app_remote.c
 *  Description  : This file contains Remote control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-07 11:26:15
 */


#include "sys_const.h"
#include "protocol_common.h"
#include "app_remote.h"
#include "app_referee.h"
#include "app_communicate.h"
#include "module_shoot.h"
#include "module_gimbal.h"
#include "module_chassis.h"
#include "module_referee.h"
#include "periph_servo.h"
#include "periph_pc_comm.h"
#include "cmsis_os.h"
#include "app_autoaim.h"
#define REMOTE_TASK_PERIOD  1	//定义遥控器任务的执行周期�?1ms
#define ENCODER_LIMIT 500		//定义编码器角度的突变阈值为 500
/* PC(0x02) 导航输入超时：超时后停止�?dh_target 覆盖底盘 */
/* 左上+右中：连续丢目标满此时长(�?Remote 周期�?才切扫描；周�?1ms 时即�?200ms */
#define XTL_SCAN_ENTER_LOST_FRAMES  ((200u) / (REMOTE_TASK_PERIOD))
#define XTL_SCAN_EXIT_FOUND_FRAMES   3u   /* 扫描态下连续识别满此帧数再回到跟�?*/
/* 视觉供弹：连�?raw shoot_msg 达此帧数后，再延�?VISION_FEED_DELAY_MS 才允�?FAST 供弹（给 pitch 跟踪时间�?*/
#define VISION_FEED_RAW_CONSEC_FRAMES  5u
#define VISION_FEED_DELAY_MS          500u

uint32_t game_state_cnt;
/* 左上+右中 跟踪态：视觉供弹延时状�?*/
static uint8_t s_xtl_vis_feed_consec = 0;
static uint8_t s_xtl_vis_feed_armed = 0;
static uint32_t s_xtl_vis_feed_start_ms = 0;
/* 左中+右中：同上，独立一套避免两种模式互相污�?*/
static uint8_t s_lm_rm_vis_feed_consec = 0;
static uint8_t s_lm_rm_vis_feed_armed = 0;
static uint32_t s_lm_rm_vis_feed_start_ms = 0;

/** raw_shoot_msg：视觉协议每�?shoot_msg；连�?5 帧非 0 后记时，�?VISION_FEED_DELAY_MS 返回 1 */
static uint8_t VisionFeed_UpdateAndDelayOk(uint8_t raw_shoot_msg,
    uint8_t *consec, uint8_t *armed, uint32_t *start_ms)
{
    if (raw_shoot_msg) {
        if (*consec < 255u) (*consec)++;
    } else {
        *consec = 0;
        *armed = 0;
    }
    if (*consec >= VISION_FEED_RAW_CONSEC_FRAMES) {
        if (!*armed) {
            *armed = 1;
            *start_ms = HAL_GetTick();
        }
    }
    return (uint8_t)(*armed && (HAL_GetTick() - *start_ms >= VISION_FEED_DELAY_MS));
}
Remote_RemoteControlTypeDef Remote_remoteControlData;
Math_SlopeParamTypeDef Remote_ChassisFBSlope;
float last_encoder_angle = 0.0f;
float encoder_angle = 0.0f;

float get_abs(float a)
{
    return (a >= 0.0f) ? a : -a;
}

float limit_siqu(float last_angle, float angle)
{
    if (get_abs(angle - last_angle) <= ENCODER_LIMIT) {
        return last_angle;
    }
    return angle;
}
/* ---------------- Vision Yaw Anti-Jitter (reference-layer smoothing) ----------------
 * 目标：把视觉 yaw_predict（偏�?误差的增量，int16）转换为“本周期参考yaw_ref增量”，
 *      再叠加到 buscomm->yaw_ref 上，从源头减少视觉毛刺导致的 yaw 抖动�? *
 * 单位约定�? *  - yaw_predict * 0.01 => 偏差（deg�? *  - inc_raw = bias_deg * gain => 本周�?yaw_ref 增量（deg/1ms�? *
 * 调参建议（按优先级）�? *  1) deadband：偏差小到一定程度就不修正（单位：deg�? *  2) inc_max：限制单次参考增量，防止某一帧毛刺导致跳变（单位：deg/1ms�? *  3) lp_alpha：一阶低通系数（0~1�? */
#define YAW_VISION_BIAS_DEADBAND_DEG   (0.02f)
#define YAW_VISION_INC_MAX_DEG_PER_MS (0.05f)
#define YAW_VISION_LP_ALPHA            (0.20f)
/* gain：对应原工程�?yaw_predict*0.01*gain �?gain 系数
 * - RemoteShooter/手动/键鼠�?.004f
 * - 视觉扫描 get_shoot_msg==1�?.002f
 */
#define YAW_VISION_GAIN_0P004         (0.004f)
#define YAW_VISION_GAIN_0P002         (0.002f)

static float g_yaw_inc_filt_0p004 = 0.0f;
static float g_yaw_inc_filt_0p002 = 0.0f;
static float VisionYaw_IncFiltered_Internal(int16_t yaw_predict_raw, float gain, float *p_filt_state)
{
    float bias_deg = (float)yaw_predict_raw * 0.01f;
    float inc_raw = 0.0f;

    // 1) Deadband: 小偏差直接当�?0
    if (get_abs(bias_deg) >= YAW_VISION_BIAS_DEADBAND_DEG)
    {
        // 2) Gain: 偏差 -> 本周�?yaw_ref 增量
        inc_raw = bias_deg * gain;

        // 3) Increment limit: 限制单次增量，防毛刺跳变
        if (inc_raw > YAW_VISION_INC_MAX_DEG_PER_MS)  inc_raw = YAW_VISION_INC_MAX_DEG_PER_MS;
        if (inc_raw < -YAW_VISION_INC_MAX_DEG_PER_MS) inc_raw = -YAW_VISION_INC_MAX_DEG_PER_MS;
    }

    // 4) One-pole low-pass: inc_raw -> inc_filt
    if (p_filt_state != NULL)
    {
        *p_filt_state = (*p_filt_state) * (1.0f - YAW_VISION_LP_ALPHA) + inc_raw * YAW_VISION_LP_ALPHA;
        return *p_filt_state;
    }

    return inc_raw;
}

/**
  * @brief          Remote task
  * @param          NULL
  * @retval         NULL
  */
void Remote_Task(void const * argument) {

    forever {
        Remote_ControlCom();
      osDelay(REMOTE_TASK_PERIOD);
    }
}


/**
  * @brief      Remote Control Init
  * @param      NULL
  * @retval     NULL
  */
void Remote_RemotrControlInit() {
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    
    Math_InitSlopeParam(&Remote_ChassisFBSlope, MOUSE_CHASSIS_ACCELERATE, MOUSE_CHASSIS_ACCELERATE);
}


/**
  * @brief      Gets the pointer to the remote control data object
  * @param      NULL
  * @retval     Pointer to remote control data object
  */
Remote_RemoteControlTypeDef* Remote_GetControlDataPtr() {
    return &Remote_remoteControlData;
}


/**
* @brief      Remote control command
* @param      NULL
* @retval     NULL
*/
Remote_RemoteDataTypeDef *testdata;
void Remote_ControlCom() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
		testdata = data;
    control_data->pending = 1;

    /* 严重安全保护机制补丁：如果通讯桥接超时或云台那边判定遥控器掉线，必须强切输出停�?*/
    if (Remote_IsRemoteOffline() || data->state == Remote_STATE_LOST) {
        Chassis_SetChassisMode(Chassis_NULL);
        Chassis_SetChassisRef(0.0f, 0.0f, 0.0f);
        Shooter_ChangeShooterMode(Shoot_NULL);
        Shooter_ChangeFeederMode(Feeder_NULL);
        control_data->pending = 0;
        return;
    }

    switch (data->remote.s[0]) {
    /*    左侧三位开关控制模�?  */
        case Remote_SWITCH_UP: {
            /* 左侧三位开关为上时是常规遥控模�?(右上底盘NULL,右中XTL,右下FOLLOW)*/
            Remote_RemoteProcess();
            break;
        }
        case Remote_SWITCH_MIDDLE: {
            /* 左侧三位开关为中时是固定给底盘SEP模式,根据右侧三位开关不同位置切换供弹射击模�?*/
            Remote_RemoteShooterModeSet();
            break;
        }
        case Remote_SWITCH_DOWN: {
            /*左侧三位开关为下时也是常规遥控模式(右上底盘NULL,右中XTL,右下FOLLOW)  */
           Remote_RemoteProcess();
            break;
        }
        default:
            break;
    }

    control_data->pending = 0;
}


/**
* @brief      Mouse shoot mode set
* @param      NULL
* @retval     NULL
*/
int test_count;
void Remote_MouseShooterModeSet() {
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();

    // Prevent launching without opening the friction wheel
//    if ((shooter->shooter_mode != Shoot_REFEREE) || (fabs(Motor_ShootLeftMotor.encoder.speed) <= 30) || (fabs(Motor_ShootRightMotor.encoder.speed) <= 30)) {
//        Shooter_ChangeFeederMode(Feeder_FINISH);
//        return;
//    }
    if ((fabs(Motor_ShootLeftMotor.encoder.speed) <= 30) || (fabs(Motor_ShootRightMotor.encoder.speed) <= 30)) {
        Shooter_ChangeFeederMode(Feeder_FINISH);
        return;
    }

    static int count_mouse_L = 0;
    if (data->mouse.l == 1) {
        count_mouse_L++;
        if (count_mouse_L >= 50) {
            Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
            count_mouse_L = 50;
        }
    }
    else {
        if (0 < count_mouse_L && count_mouse_L < 50) {
            Shooter_SingleShootReset();
            Shooter_ChangeFeederMode(Feeder_SINGLE);
        }
        else Shooter_ChangeFeederMode(Feeder_FINISH);
        count_mouse_L = 0;
    }
		
		test_count = count_mouse_L;
}


/**
* @brief      Remote shoot mode set
* @param      NULL
* @retval     NULL
*/
void Remote_RemoteShooterModeSet() {


		Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
		Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
		

    switch (data->remote.s[1]) {
    /*      left switch control mode   */
        case Remote_SWITCH_UP: {
            /* 左中前提下右上是射击供弹模式NULL  */
            Shooter_ChangeShooterMode(Shoot_NULL);
            Shooter_ChangeFeederMode(Feeder_NULL);
            s_lm_rm_vis_feed_consec = 0;
            s_lm_rm_vis_feed_armed = 0;
            break;
        }
        case Remote_SWITCH_MIDDLE: {
            /* 左中前提下右中是切换射击供弹模式  */
            Shooter_ChangeShooterMode(Shoot_FAST);
					{
						uint8_t raw_vis = (visionDataGet.shoot_msg != 0);
						uint8_t feed_delay_ok = VisionFeed_UpdateAndDelayOk(raw_vis,
						    &s_lm_rm_vis_feed_consec, &s_lm_rm_vis_feed_armed, &s_lm_rm_vis_feed_start_ms);
						if (feed_delay_ok) {
							Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
						} else if (get_shoot_msg(&visionDataGet) == 1) {
							/* 已识别但未满�? �?+ 1s」，先不连续供弹 */
							Shooter_ChangeFeederMode(Feeder_FINISH);
						} else {
							Shooter_ChangeFeederMode(Feeder_LOW_CONTINUE);
						}
					}
            break;
        }
        case Remote_SWITCH_DOWN: {
            /* 左中前提下右下是切换射击快速供弹模�? */
            Shooter_ChangeShooterMode(Shoot_FAST);
            Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
            s_lm_rm_vis_feed_consec = 0;
            s_lm_rm_vis_feed_armed = 0;
//            if ((PID_GetPIDFdb(&shooter->shootLeftPID) >= 30) && (PID_GetPIDFdb(&shooter->shootRightPID) <= -30)) {
//                Shooter_ChangeFeederMode(Feeder_REFEREE);
//            }
//            else Shooter_ChangeFeederMode(Feeder_FINISH);
            break;
        }
        default:
            break;
    }
		
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
		gimbalpitch->output_state = 1;
		gimbalyaw->output_state = 1;
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->yaw_ref += (float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN +
                         VisionYaw_IncFiltered_Internal(visionDataGet.yaw_angle.yaw_predict, YAW_VISION_GAIN_0P004, &g_yaw_inc_filt_0p004);
		GimbalYaw_SetYawRef(buscomm->yaw_ref);
    float pitch_ref;
    pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF;//+ (float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
    float cospitch = pitch_ref*PI/180.0f;
	GimbalPitch_SetPitchRef(cospitch);
		
		Chassis_SetChassisMode(Chassis_GIMBAL);
		Chassis_SetChassisRef((float)data->remote.ch[1]  , (float)data->remote.ch[0] , 0);
		
		
}


/**
* @brief      Remote control process
* @param      NULL
* @retval     NULL
*/
void Remote_RemoteProcess() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
	  TargetData *dh_target = Dh_GetDhDataPtr();
    /* 左中右中 才用 lm_rm 供弹延时；左开关为�?下时走本函数，清零避免切回左中时沿用旧状�?*/
    s_lm_rm_vis_feed_consec = 0;
    s_lm_rm_vis_feed_armed = 0;

    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
		gimbalpitch->output_state = 0;
		gimbalyaw->output_state = 0;
		
    switch (data->remote.s[1]) {
    /*     右开关决定模�?  */
        case Remote_SWITCH_UP: {//当左开关为�?右开关为上时,空输出NULL
					  Shooter_ChangeShooterMode(Shoot_NULL);
            Shooter_ChangeFeederMode(Feeder_NULL);
            s_xtl_vis_feed_consec = 0;
            s_xtl_vis_feed_armed = 0;
	          gimbalpitch->output_state = 0;
	          gimbalyaw->output_state = 0;
						Chassis_SetChassisMode(Chassis_NULL);
					Chassis_SetChassisRef(0.0f, 0.0f, 0.0f);
//				buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN + (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f);
//GimbalYaw_SetYawRef(buscomm->yaw_ref);
//float pitch_ref;
//pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF; //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
//GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));
            break;
        }

//				case Remote_SWITCH_MIDDLE:  
//				{
//	       gimbalpitch->output_state = 1;
//	       gimbalyaw->output_state = 1;
//	       Chassis_SetChassisMode(Chassis_FOLLOW);
//				 last_encoder_angle=encoder_angle;
//				 encoder_angle=(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET);
//	       Chassis_SetChassisRef((float)dh_target->linear_x*Const_Chasiss_Navigate_linear_x  , -(float)dh_target->linear_y*Const_Chasiss_Navigate_linear_y ,limit_siqu(last_encoder_angle,encoder_angle)); //(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET));
//         buscomm->yaw_ref += Gimbal_LimitYaw((float)dh_target->angular_z * Const_Chasiss_Navigate_angular_z ); 
//         float pitch_ref;
//         pitch_ref = Gimbal_LimitYaw((float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF + (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f); //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
//         GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref)); 
//				break;
//				}
												// 注释掉的代码（已废弃�?//    case Remote_SWITCH_MIDDLE:
//        {
//	      gimbalpitch->output_state = 1;
//	      gimbalyaw->output_state = 1;
//	      Chassis_SetChassisMode(Chassis_FOLLOW);
//				last_encoder_angle=encoder_angle;
//				encoder_angle=(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET);
//	      Chassis_SetChassisRef((float)data->remote.ch[1] , (float)data->remote.ch[0] ,limit_siqu(last_encoder_angle,encoder_angle)); //(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET));
//        buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN + (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f);
//		    GimbalYaw_SetYawRef(buscomm->yaw_ref);
//        float pitch_ref;
//        pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF; //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
//        GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));  
//					break;
//        }// 注释掉的代码 2（跟随模式）
				
		case Remote_SWITCH_MIDDLE: /* 左上+右中：小陀�?视觉扫描；底�?XTL（车身系，见 Chassis_XTL 实现�?*/
			{
//			if(Referee_RefereeData.game_state==0x04)
//			{
			  Shooter_ChangeShooterMode(Shoot_FAST);
				if(Referee_RefereeData.game_state==4)
				{
				game_state_cnt++;
				}
		    float yaw_autoadd;
		    static float pitch_autoadd = 20.0f;
			static int add = 0;
			uint8_t state;
			uint8_t last_state;
				/* 滞回：短�?get_shoot_msg==0（如击发闪断）不立刻走扫�?yaw_ref，避免与视觉跟踪打架 */
				static uint8_t s_xtl_scan_aim_locked = 0;
				static uint16_t s_xtl_scan_lost_cnt = 0;
				static uint16_t s_xtl_scan_found_cnt = 0;
				{
					uint8_t raw_xtl_aim = get_shoot_msg(&visionDataGet);
					if (raw_xtl_aim == 0) {
						if (s_xtl_scan_lost_cnt < 0xFFFFu) s_xtl_scan_lost_cnt++;
						s_xtl_scan_found_cnt = 0;
					} else {
						if (s_xtl_scan_found_cnt < 0xFFFFu) s_xtl_scan_found_cnt++;
						s_xtl_scan_lost_cnt = 0;
					}
					if (s_xtl_scan_aim_locked) {
						if (s_xtl_scan_lost_cnt >= XTL_SCAN_ENTER_LOST_FRAMES)
							s_xtl_scan_aim_locked = 0;
					} else {
						if (s_xtl_scan_found_cnt >= XTL_SCAN_EXIT_FOUND_FRAMES)
							s_xtl_scan_aim_locked = 1;
					}
				}
				if(s_xtl_scan_aim_locked)
				{ 
					{
						uint8_t raw_vis = (visionDataGet.shoot_msg != 0);
						uint8_t feed_delay_ok = VisionFeed_UpdateAndDelayOk(raw_vis,
						    &s_xtl_vis_feed_consec, &s_xtl_vis_feed_armed, &s_xtl_vis_feed_start_ms);
						if (feed_delay_ok) {
							Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
						} else {
							Shooter_ChangeFeederMode(Feeder_FINISH);
						}
					}
				 gimbalpitch->output_state = 1;
	       gimbalyaw->output_state = 1;			
	       Chassis_SetChassisMode(Chassis_XTL);
				 last_encoder_angle=encoder_angle;
				 encoder_angle = Comm_BoardLinkGimbalRelYawDeg();
	       Chassis_SetChassisRef((float)data->remote.ch[1], (float)data->remote.ch[0], CHASSIS_XTL_WZ);	
				 buscomm->yaw_ref += Gimbal_LimitYaw(VisionYaw_IncFiltered_Internal(visionDataGet.yaw_angle.yaw_predict, YAW_VISION_GAIN_0P002, &g_yaw_inc_filt_0p002));
         GimbalYaw_SetYawRef(buscomm->yaw_ref);
				 float pitch_ref;
         pitch_ref = (float)visionDataGet.pitch_angle.pitch_predict*3.1415926/180*0.01f*0.002f;//*(-1)
         GimbalPitch_SetPitchRef(Gimbal_LimitPitch(pitch_ref)); 					 
				}
			else
  {
         s_xtl_vis_feed_consec = 0;
         s_xtl_vis_feed_armed = 0;
         Shooter_ChangeFeederMode(Feeder_FINISH);
         /* 小陀螺扫描：XTL 模式，车身系平移 */
         gimbalpitch->output_state = 1;
         gimbalyaw->output_state = 1;
         Chassis_SetChassisMode(Chassis_XTL);
         last_encoder_angle = encoder_angle;
         encoder_angle = Comm_BoardLinkGimbalRelYawDeg();
         Chassis_SetChassisRef((float)data->remote.ch[1], (float)data->remote.ch[0], CHASSIS_XTL_WZ);
         {
           yaw_autoadd = Const_YAW_AUTOADD;
         }
         /* 识别丢失 -> 进入视觉扫描：清空滤波状态，避免切回时旧 inc 造成额外跳变 */
         g_yaw_inc_filt_0p004 = 0.0f;
         g_yaw_inc_filt_0p002 = 0.0f;
         buscomm->yaw_ref += Gimbal_LimitYaw(yaw_autoadd * -Const_Vision_YAW_GAIN);
         GimbalYaw_SetYawRef(buscomm->yaw_ref);

         /* 达妙 Pitch：在 [-0.2, +0.2] 之间循环扫描
          * 目标是：-0.2 -> +0.2 -> -0.2 �?1s
          * Remote task 周期 REMOTE_TASK_PERIOD=1ms，因此半�?0.5s �?500 次循�?          * 单步增量 0.4 / 500 �?0.0008
          * 翻面依据�?pitch_ref_smooth（现在的控制输出目标 pos 基本等于它）�?         */
         static int pitch_dir = 1; // +1 上扫�?1 下扫
         const float P_MIN = -0.2f;
         const float P_MAX =  0.2f;
         const float margin = 0.005f;
         const float step = 0.0008f;

         float p_smooth = gimbalpitch->pitch_ref_smooth;
         if (p_smooth >= (P_MAX - margin)) {
           pitch_dir = -1;
         } else if (p_smooth <= (P_MIN + margin)) {
           pitch_dir = +1;
         }

         float inc = (float)pitch_dir * step;
         if (p_smooth + inc > P_MAX) inc = P_MAX - p_smooth;
         if (p_smooth + inc < P_MIN) inc = P_MIN - p_smooth;

         GimbalPitch_SetPitchRef(inc);
         /* PC 导航：仍�?XTL，dh_target 按车身系 vx/vy/wz 解释（与模块�?Chassis_XTL 一致） */
         {
           uint8_t pc_nav_online = PC_Comm_IsOnline();

           if (!pc_nav_online) {
             /* 超时则清零，防止旧速度残留导致持续覆盖 */
             dh_target->linear_x = 0.0f;
             dh_target->linear_y = 0.0f;
             dh_target->angular_z = 0.0f;
           }

           float dx = (float)dh_target->linear_x, dy = (float)dh_target->linear_y, dz = (float)dh_target->angular_z;//导航的xyz数据
           const float nav_thresh = 0.01f;
          if (pc_nav_online && (dx*dx + dy*dy + dz*dz > nav_thresh * nav_thresh))
          {
            Chassis_SetChassisMode(Chassis_XTL);
            Chassis_SetChassisRef(dx * Const_Chasiss_Navigate_linear_x,
                                  -dy * Const_Chasiss_Navigate_linear_y,
                                  dz * Const_Chasiss_Navigate_angular_z);
          }
         }
       }
					break;
        }// 自动瞄准模式（视觉自瞄）
								
//        case Remote_SWITCH_DOWN: {
//            /* left switch down is slow shooting   */
//	          gimbalpitch->output_state = 1;
//	          gimbalyaw->output_state = 1;
//		        Chassis_SetChassisYawAngle(Motor_YawMotor.encoder.limited_angle,CHASSIS_YAW_ANGLE_OFFSET);
//		        Chassis_SetChassisMode(Chassis_FOLLOW);
//		        Chassis_SetChassisRef((float)data->remote.ch[1]  , (float)data->remote.ch[0] , limit_siqu(last_encoder_angle,encoder_angle));
//						buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN + (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f);
//		GimbalYaw_SetYawRef(buscomm->yaw_ref);
//    float pitch_ref;
//    pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF; //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
//    GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));
//							
//            break;
//        }// 注释掉的代码 3（慢速射击模式）
				case Remote_SWITCH_DOWN: {
            /* left switch down is slow shooting   */
            s_xtl_vis_feed_consec = 0;
            s_xtl_vis_feed_armed = 0;
	      gimbalpitch->output_state = 1;
	      gimbalyaw->output_state = 1;
	      Chassis_SetChassisMode(Chassis_FOLLOW);
				last_encoder_angle=encoder_angle;
				encoder_angle = Comm_BoardLinkGimbalRelYawDeg();
	      Chassis_SetChassisRef((float)data->remote.ch[1] , (float)data->remote.ch[0] ,limit_siqu(last_encoder_angle,encoder_angle)); //(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET));
        buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN +
                                              VisionYaw_IncFiltered_Internal(visionDataGet.yaw_angle.yaw_predict, YAW_VISION_GAIN_0P004, &g_yaw_inc_filt_0p004));
		    GimbalYaw_SetYawRef(buscomm->yaw_ref);
        float pitch_ref;
        pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF; //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
        GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));
            break;
        }// 手动控制模式（跟随底盘）

        default:
            break;
    }
	
    
}


/**
* @brief      KeyMouse control process
* @param      NULL
* @retval     NULL
*/
void Remote_KeyMouseProcess() { 
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    GimbalPitch_GimbalPitchTypeDef *gimbal = GimbalPitch_GetGimbalPitchPtr();
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    
	
		//chassis control
		float chassis_vx;
		float chassis_vy;
		
		if(data->key.w == 1)
		{
			chassis_vx = 200.0f;
		}	
		else if(data->key.s == 1)
		{
			chassis_vx = -200.0f;
		}
		else
		{
			chassis_vx = 0.0f;
		}
		
		if(data->key.a == 1)
		{
			chassis_vy = -200.0f;
		}
		else if(data->key.d == 1)
		{
			chassis_vy = 200.0f;
		}
		else
		{
			chassis_vy = 0.0f;
		}
		
		if(data->key.shift == 1)
		{
		  Chassis_SetChassisYawAngle(Comm_BoardLinkGimbalRelYawDeg(), 0.0f);
		  Chassis_SetChassisMode(Chassis_XTL);
		  Chassis_SetChassisRef(chassis_vx  , chassis_vy , CHASSIS_XTL_WZ);
		}
		else if(data->key.shift == 0)
		{
			Chassis_SetChassisMode(Chassis_FOLLOW);
			Chassis_SetChassisRef(chassis_vx  , chassis_vy , Comm_BoardLinkGimbalRelYawDeg());	
		}

		if(data->key.q == 1)
		{
			Servo_SetServoAngle(&Servo_ammoContainerCapServo, 90);
		}
		else if(data->key.q == 0)
		{
			Servo_SetServoAngle(&Servo_ammoContainerCapServo, 0);
		}
	
		
		//autoaim control
		float autoaim_yaw;
		float autoaim_pitch;
		if(data->mouse.r == 1)
		{
			autoaim_yaw = VisionYaw_IncFiltered_Internal(visionDataGet.yaw_angle.yaw_predict, YAW_VISION_GAIN_0P004, &g_yaw_inc_filt_0p004);
			autoaim_pitch = (float)visionDataGet.pitch_angle.pitch_predict*3.1415926/180*0.01f*0.002f;
		}
		else if(data->mouse.r == 0)
		{
			autoaim_yaw = 0.0f;
			autoaim_pitch = 0.0f;
			/* 鼠标未按�?-> 自动瞄准关闭：清空视�?yaw 滤波状�?*/
			g_yaw_inc_filt_0p004 = 0.0f;
		}
		
		
		//gimbal control
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
		gimbalpitch->output_state = 1;
		gimbalyaw->output_state = 1;
    buscomm->yaw_ref += Gimbal_LimitYaw((float)data->mouse.x * -MOUSE_YAW_ANGLE_TO_FACT + autoaim_yaw);
		GimbalYaw_SetYawRef(buscomm->yaw_ref);
    float pitch_ref;
    pitch_ref = ((float)data->mouse.y * -MOUSE_PITCH_ANGLE_TO_FACT - autoaim_pitch);
    GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));

		//shoot control(fric)
    if (data->key.f == 1)      
        Shooter_ChangeShooterMode(Shoot_FAST);
    if (data->key.g == 1)      
        Shooter_ChangeShooterMode(Shoot_NULL);

}

