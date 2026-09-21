/*
 *  Project      : Infantry_Neptune
 * 
 *  file         : gim_shoot_ctrl.c
 *  Description  : This file contains Shooter control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-07 09:29:25
 */

#include "module_shoot.h"
#include "sys_const.h"
#include "cmsis_os.h"

#include "periph_referee.h"
#include "util_gpio.h"
#include "sys_robot_actuators.h"

Shoot_StatusTypeDef Shooter_ShooterControl;

/**
  * @brief      shooter control initialization
  * @param      NULL
  * @retval     NULL
  */
void Shooter_InitShooter() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    
    shooter->shooter_control = 1;

    shooter->feeder_mode = Feeder_NULL;
    shooter->heat_ctrl.shooter_17mm_cooling_heat = 0;
    shooter->heat_ctrl.shooter_17mm_cooling_rate = 0;

    shooter->shooter_mode = Shoot_NULL;
    shooter->shoot_speed.feeder_shoot_speed = 0;
    shooter->shoot_speed.left_shoot_speed = 0;
    shooter->shoot_speed.right_shoot_speed = 0;
    
    shooter->shooter_speed_15mpers = Const_Shooter15mpers;
    shooter->shooter_speed_18mpers = Const_Shooter18mpers;
    shooter->shooter_speed_22mpers = Const_Shooter22mpers;
    shooter->shooter_speed_30mpers = Const_Shooter30mpers;

    PID_InitPIDParam(&shooter->shootLeftPIDParam, Const_ShootLeftParam[0][0], Const_ShootLeftParam[0][1], Const_ShootLeftParam[0][2], Const_ShootLeftParam[0][3], 
                    Const_ShootLeftParam[0][4], Const_ShootLeftParam[1][0], Const_ShootLeftParam[1][1], Const_ShootLeftParam[2][0], Const_ShootLeftParam[2][1], 
                    Const_ShootLeftParam[3][0], Const_ShootLeftParam[3][1], PID_POSITION);      
    PID_InitPIDParam(&shooter->shootRightPIDParam, Const_ShootRightParam[0][0], Const_ShootRightParam[0][1], Const_ShootRightParam[0][2], Const_ShootRightParam[0][3], 
                    Const_ShootRightParam[0][4], Const_ShootRightParam[1][0], Const_ShootRightParam[1][1], Const_ShootRightParam[2][0], Const_ShootRightParam[2][1], 
                    Const_ShootRightParam[3][0], Const_ShootRightParam[3][1], PID_POSITION); 
    PID_InitPIDParam(&shooter->feedAngPIDParam, Const_FeedAngParam[0][0], Const_FeedAngParam[0][1], Const_FeedAngParam[0][2], Const_FeedAngParam[0][3], 
                    Const_FeedAngParam[0][4], Const_FeedAngParam[1][0], Const_FeedAngParam[1][1], Const_FeedAngParam[2][0], Const_FeedAngParam[2][1], 
                    Const_FeedAngParam[3][0], Const_FeedAngParam[3][1], PID_POSITION); 
    PID_InitPIDParam(&shooter->feedSpdPIDParam, Const_FeedSpdParam[0][0], Const_FeedSpdParam[0][1], Const_FeedSpdParam[0][2], Const_FeedSpdParam[0][3], 
                    Const_FeedSpdParam[0][4], Const_FeedSpdParam[1][0], Const_FeedSpdParam[1][1], Const_FeedSpdParam[2][0], Const_FeedSpdParam[2][1], 
                    Const_FeedSpdParam[3][0], Const_FeedSpdParam[3][1], PID_POSITION); 
}


/**
  * @brief      Shooter control
  * @param      NULL
  * @retval     NULL
  */
void Shooter_Control() {


    Shooter_ShootControl();
    


    Shooter_ShooterMotorOutput();
}


/**
  * @brief      Gets the pointer to the shooter control data object
  * @param      NULL
  * @retval     Pointer to shooter control data object
  */
Shoot_StatusTypeDef* Shooter_GetShooterControlPtr() {
    return &Shooter_ShooterControl;
}


/**
  * @brief      Get speed offset variable    (150 means offset is 0)
  * @param      NULL
  * @retval     NULL
  */
float Shooter_GetShootSpeedOffset() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    Referee_RefereeDataTypeDef *referee = Referee_GetRefereeDataPtr();
    float offset_speed;

    switch (referee->shooter_heat0_speed_limit) {
        case 15: 
            offset_speed = shooter->shoot_speed_offset.speed_15mm_offset;
            break;
        case 18:          
            offset_speed = shooter->shoot_speed_offset.speed_18mm_offset;
            break;
        case 22:
            offset_speed = shooter->shoot_speed_offset.speed_22mm_offset;
            break;
        case 30:
            offset_speed = shooter->shoot_speed_offset.speed_30mm_offset;
            break;
        default:
            offset_speed = 15.0f;
            break;
    }
    return offset_speed;
}


/**
  * @brief      Change frequent mode
  * @param      mode: Shooter mode
  * @retval     NULL
  */
void Shooter_ChangeShooterMode(Shoot_ShooterModeEnum mode) {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
//    Referee_RefereeDataTypeDef *referee = Referee_GetRefereeDataPtr();
//    
//    if (referee->mains_power_shooter_output == 1)
//        shooter->shooter_mode = mode;
//    else 
//        shooter->shooter_mode = Shoot_NULL;
	  shooter->shooter_mode = mode;
}


/**
  * @brief      Change shooter mode
  * @param      mode: Feeder mode
  * @retval     NULL
  */
void Shooter_ChangeFeederMode(Shoot_FeederModeEnum mode) {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    if (shooter->feeder_mode == Feeder_LOCKED_ROTOR) return;
    shooter->last_feeder_mode = shooter->feeder_mode;
    shooter->feeder_mode = mode;
    if ((shooter->feeder_mode != shooter->last_feeder_mode) &&
       ((shooter->last_feeder_mode == Feeder_LOW_CONTINUE) ||
        (shooter->last_feeder_mode == Feeder_FAST_CONTINUE) ||
        (shooter->last_feeder_mode == Feeder_REFEREE))) {
        shooter->feeder_mode = Feeder_FINISH;
        Shooter_AngleCorrect();
    }
}


/**
  * @brief      Set referee shooter speed
  * @param      NULL
  * @retval     NULL
  */
float Shooter_GetRefereeSpeed() {
    Referee_RefereeDataTypeDef *referee = Referee_GetRefereeDataPtr();
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();

    float speed;
    switch (referee->shooter_heat0_speed_limit) {
        case 15: 
            speed = shooter->shooter_speed_15mpers;
            break;
        case 18:
            speed = shooter->shooter_speed_18mpers;
            break;
        case 22:
            speed = shooter->shooter_speed_22mpers;
            break;
        case 30:
            speed = shooter->shooter_speed_30mpers;
            break;
        default:
            speed = shooter->shooter_speed_15mpers;
            break;
    }

    return speed;
}


/**
  * @brief      Updata control data
  * @param      NULL
  * @retval     NULL
  */
void Shooter_UpdataControlData() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    Referee_RefereeDataTypeDef *referee = Referee_GetRefereeDataPtr();
	
    shooter->heat_ctrl.shooter_17mm_cooling_heat = (float)referee->shooter_heat0;
    shooter->heat_ctrl.shooter_17mm_cooling_rate = (float)referee->shooter_heat0_cooling_limit;
    
    Shooter_FeederMotorLockedJudge();
}


/**
  * @brief      Set feeder motor speed
  * @param      speed: Feeder motor speed ref
  * @retval     NULL
  */
void Shooter_SetFeederSpeed(float speed) {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();

    shooter->shoot_speed.feeder_shoot_speed = speed;
}


/**
  * @brief      Set shooter motor speed
  * @param      speed: shooter motor speed ref
  * @retval     NULL
  */
void Shooter_SetShooterSpeed(float speed) {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    
    shooter->shoot_speed.left_shoot_speed  = speed;
    shooter->shoot_speed.right_shoot_speed = -speed;
	
}


/**
  * @brief      Force change shooter mode
  * @param      mode: Feeder mode
  * @retval     NULL
  */
void Shooter_ForceChangeFeederMode(Shoot_FeederModeEnum mode) {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    shooter->feeder_mode = mode;
}


/**
  * @brief      Motor locked rotor judge
  * @param      NULL
  * @retval     NULL
  */
void Shooter_FeederMotorLockedJudge() {
  Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();

  static int count = 0;
  if (shooter->feeder_mode != Feeder_LOCKED_ROTOR) {
    Actuator_FeedbackTypeDef feeder_feedback = Actuator_MotorGetFeedback(
        RobotActuators_GetMotor(ROBOT_MOTOR_FEEDER));
    if ((fabs(feeder_feedback.effort) >= Const_ShooterLockedCurrent) &&
       (fabs(feeder_feedback.velocity) <= Const_ShooterLockedSpeed)) {
        count ++;
        if (count > Const_ShooterLockedTime) {
            Shooter_ForceChangeFeederMode(Feeder_LOCKED_ROTOR);
        }
    }
    else count = 0;
  }
}


/**
  * @brief      Motor locked handle
  * @param      NULL
  * @retval     NULL
  */
void Shooter_MotorLockedHandle() {
	  static int count_reverse = 0;
	  Shooter_SetFeederSpeed(Const_ShooterLockedReverseSpeed);
	  count_reverse++;
	  if(count_reverse >= Const_ShooterRelockedTime) {
		    count_reverse = 0;
        Shooter_ForceChangeFeederMode(Feeder_NULL);
	}
}


/**
  * @brief      Correct stop angle
  * @param      NULL
  * @retval     NULL
  */
void Shooter_AngleCorrect() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    PID_SetPIDRef(&shooter->feedAngPID, PID_GetPIDFdb(&shooter->feedAngPID));
}

void Shooter_RealAngleCorrect() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    PID_SetPIDRef(&shooter->feedAngPID, ((int)(PID_GetPIDFdb(&shooter->feedAngPID) + 40.0f) / 45) * 45);
}


/**
  * @brief      Shooter heat control
  * @param      NULL
  * @retval     pid_num
  */
uint8_t Shooter_HeatCtrl() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();

    if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat)  >= Const_HeatCtrlFastLimit) {   // sufficient heat remain, fast shooting
        shooter->heat_ctrl.current_speed = Const_FeederFastSpeed;
        shooter->heat_ctrl.current_pidnum = 1;
        Shooter_SetFeederSpeed(shooter->heat_ctrl.current_speed);
        shooter->heat_ctrl.heat_tracking = 0;
    }
    else {
        if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat)  >= Const_HeatCtrlSlowLimit) {
            shooter->heat_ctrl.current_speed = Const_FeederSlowSpeed;
            shooter->heat_ctrl.current_pidnum = 1;
            Shooter_SetFeederSpeed(shooter->heat_ctrl.current_speed);
            shooter->heat_ctrl.heat_tracking = 0;
        }
        else {
            if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) <= Const_HeatCtrlWaitLimit) {
            shooter->heat_ctrl.current_speed = Const_FeederWaitSpeed;
            shooter->heat_ctrl.current_pidnum = 1;
            Shooter_SetFeederSpeed(shooter->heat_ctrl.current_speed);
            shooter->heat_ctrl.heat_tracking = 0;
           }
           else {
                if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) <= Const_HeatCtrlStopLimit) {
                    shooter->heat_ctrl.heat_tracking = 0;
                    Shooter_AngleCorrect();
                    shooter->heat_ctrl.current_pidnum = 2;
                }
            }
        }
    }
    return shooter->heat_ctrl.current_pidnum;
}


/**
  * @brief      Shooter control
  * @param      NULL
  * @retval     NULL
  */
void Shooter_ShootControl() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();

    switch (shooter->shooter_mode) {
      case Shoot_NULL:
          GPIO_Reset(LASER);	 //关闭激光(现在车上激光应该已经被淘汰了)
          Shooter_SetShooterSpeed(0);
          break;
      case Shoot_FAST:
          GPIO_Set(LASER);       //开启激光
          Shooter_SetShooterSpeed(Const_ShooterFastSpeed);
          break;
      case Shoot_SLOW:
          GPIO_Set(LASER);
          Shooter_SetShooterSpeed(Const_ShooterSlowSpeed);
          break;
      case Shoot_REFEREE:
          GPIO_Set(LASER);
          Shooter_SetShooterSpeed(Shooter_GetRefereeSpeed() + Shooter_GetShootSpeedOffset());
          break;
      default:
          break;
    }
	//下面给摩擦轮电机配置PID
    PID_SetPIDRef(&shooter->shootLeftPID, shooter->shoot_speed.left_shoot_speed);
    PID_SetPIDRef(&shooter->shootRightPID, shooter->shoot_speed.right_shoot_speed);
    PID_SetPIDFdb(&shooter->shootLeftPID, Actuator_MotorGetFeedback(
        RobotActuators_GetMotor(ROBOT_MOTOR_SHOOT_LEFT)).velocity);
    PID_SetPIDFdb(&shooter->shootRightPID, Actuator_MotorGetFeedback(
        RobotActuators_GetMotor(ROBOT_MOTOR_SHOOT_RIGHT)).velocity);
    PID_CalcPID(&shooter->shootLeftPID, &shooter->shootLeftPIDParam);
    PID_CalcPID(&shooter->shootRightPID, &shooter->shootRightPIDParam);
    (void)Actuator_MotorSetEffort(
        RobotActuators_GetMotor(ROBOT_MOTOR_SHOOT_LEFT),
        PID_GetPIDOutput(&shooter->shootLeftPID));
    (void)Actuator_MotorSetEffort(
        RobotActuators_GetMotor(ROBOT_MOTOR_SHOOT_RIGHT),
        PID_GetPIDOutput(&shooter->shootRightPID));
}


/**
  * @brief      Shooter feeder control:单发射击(角度环控制拨盘电机每次转45度,发射出一发子弹)
  * @param      NULL
  * @retval     NULL
  */
void Shooter_SingleShootCtrl() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    
    if (fabs(PID_GetPIDFdb(&shooter->feedAngPID) - PID_GetPIDRef(&shooter->feedAngPID)) > 1.0f) {//计算目标角度和实际角度的绝对差值，如果差值 > 1°，说明电机还未回到目标位置（未就绪）
        //return;     // do nothing
    }
    if (!shooter->single_shoot_done) {   // not shoot yet
        PID_AddPIDRef(&shooter->feedAngPID, 45.0f);
        PID_SetPIDFdb(&shooter->feedAngPID, Actuator_MotorGetFeedback(
            RobotActuators_GetMotor(ROBOT_MOTOR_FEEDER)).continuous_position);
        shooter->single_shoot_done = 1;
    }
}


/**
  * @brief      Shooter feeder control: reset single shooting
  * @param      NULL
  * @retval     NULL
  */
void Shooter_SingleShootReset() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    
    shooter->single_shoot_done = 0;
}


/**
  * @brief      Shooter feeder control
  * @param      NULL
  * @retval     NULL
  */
void Shooter_FeederControl() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();

    int current_pid_num = 0;
    switch (shooter->feeder_mode) {
      case Feeder_NULL:
          current_pid_num = 0;//速度pid(强制归零),停止模式
          Shooter_SetFeederSpeed(0);
          break;
      case Feeder_SINGLE:
          current_pid_num = 2;//角度pid速度pid双环
          Shooter_SingleShootCtrl();
          break;
      case Feeder_FAST_CONTINUE:
          current_pid_num = 1;//速度pid
          Shooter_SetFeederSpeed(Const_FeederFastSpeed);
          break;
      case Feeder_LOW_CONTINUE:
          current_pid_num = 1;
          Shooter_SetFeederSpeed(Const_FeederSlowSpeed);
          break;
      case Feeder_LOCKED_ROTOR:
          current_pid_num = 1;
          Shooter_MotorLockedHandle();
          break;
      case Feeder_REFEREE:
          current_pid_num = Shooter_HeatCtrl();
          break;
      case Feeder_FINISH:
          current_pid_num = 2;
          break;
      default:
          break;
    }
    if (current_pid_num == 0) {
        PID_ClearPID(&shooter->feedAngPID);
        PID_SetPIDRef(&shooter->feedSpdPID, 0);
        PID_SetPIDFdb(&shooter->feedSpdPID, 0);
        PID_CalcPID(&shooter->feedSpdPID, &shooter->feedSpdPIDParam);
        (void)Actuator_MotorSetEffort(
            RobotActuators_GetMotor(ROBOT_MOTOR_FEEDER),
            PID_GetPIDOutput(&shooter->feedSpdPID));
    }
    if (current_pid_num == 1) {
        PID_ClearPID(&shooter->feedAngPID);
        PID_SetPIDRef(&shooter->feedSpdPID, shooter->shoot_speed.feeder_shoot_speed);
        PID_SetPIDFdb(&shooter->feedSpdPID, Actuator_MotorGetFeedback(
            RobotActuators_GetMotor(ROBOT_MOTOR_FEEDER)).velocity);
        PID_CalcPID(&shooter->feedSpdPID, &shooter->feedSpdPIDParam);
        (void)Actuator_MotorSetEffort(
            RobotActuators_GetMotor(ROBOT_MOTOR_FEEDER),
            PID_GetPIDOutput(&shooter->feedSpdPID));
    }
    if (current_pid_num == 2) {
        PID_CalcPID(&shooter->feedAngPID, &shooter->feedAngPIDParam);     
        PID_SetPIDRef(&shooter->feedSpdPID, PID_GetPIDOutput(&shooter->feedAngPID));
        PID_SetPIDFdb(&shooter->feedSpdPID, Actuator_MotorGetFeedback(
            RobotActuators_GetMotor(ROBOT_MOTOR_FEEDER)).velocity);
        PID_CalcPID(&shooter->feedSpdPID, &shooter->feedSpdPIDParam);
        (void)Actuator_MotorSetEffort(
            RobotActuators_GetMotor(ROBOT_MOTOR_FEEDER),
            PID_GetPIDOutput(&shooter->feedSpdPID));
    }   
}


/**
  * @brief      Output shooter motor
  * @param      NULL
  * @retval     NULL
  */
void Shooter_ShooterMotorOutput() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();

    if (shooter->shooter_control == 1) {
        (void)Actuator_GroupFlush(
            RobotActuators_GetGroup(ROBOT_GROUP_SHOOTER));
    }
}
