/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_init.c
 *  Description  : All initialization threads
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:42:52
 *  LastEditTime : 2023-05-05 15:17:43
 */


#include "app_init.h"
#include "sys_softTimer.h"
#include "sys_dwt.h"
#include "alg_pid.h"
#include "util_can.h"
#include "periph_bmi088.h"
#include "periph_remote.h"
#include "periph_referee.h"
#include "periph_pc_comm.h"
#include "periph_servo.h"
#include "module_gimbal.h"
#include "module_shoot.h"
#include "module_chassis.h"
#include "module_referee.h"
#include "app_ins.h"
#include "app_control.h"
#include "app_communicate.h"
#include "app_board_config.h"
#include "protocol_common.h"
#include "module_power.h"
#include "sys_robot_actuators.h"

void Init_InitAll() {
    DWT_Init(168);

#if BOARD_HAS_INS
    BMI088_Init(0);
    INS_Init();
#endif

#if BOARD_HAS_REMOTE_UART
    Remote_InitRemote();
#endif

#if BOARD_HAS_CHASSIS
	//超电
	SuperCap_InitHeader(&CapHeader);
#endif

    // Motor Group init
	Can_InitFilterAndStart(&hcan1);
	Can_InitFilterAndStart(&hcan2);
    RobotActuators_Init();

#if BOARD_HAS_PC_UART
    PC_Comm_Init();// 初始化 PC 通信（上位机/视觉端）
#endif

#if BOARD_HAS_REFEREE_UART
    Referee_InitReferee();
#endif

#if BOARD_HAS_GIMBAL
    Servo_InitAllServos();// 初始化舵机
	

	//Referee_Setup();// 已移除，原裁判系统功能

	GimbalPitch_InitGimbalPitch();// 云台 Pitch 轴初始化
	GimbalYaw_InitGimbalYaw();// 云台 Yaw 轴初始化
    Shooter_InitShooter();// 发射机构初始化

#endif

#if BOARD_HAS_CHASSIS
	Chassis_InitChassis();// 底盘初始化
#endif

    Control_Init();
    Protocol_InitProtocol();
    Comm_BoardLinkInit();
}


void Init_Task(void const * argument) {      //初始化任务
    SoftTimer_StartAll();                        //启动所有软定时器
    forever {
        vTaskSuspend(Init_TaskHandleHandle);             //挂起初始化任务
      osDelay(25);
    }
}
