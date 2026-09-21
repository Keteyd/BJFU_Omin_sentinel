#ifndef SYS_ROBOT_ACTUATORS_H
#define SYS_ROBOT_ACTUATORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"
#include "lib_actuator.h"
#include <stdint.h>

typedef enum {
    ROBOT_MOTOR_CHASSIS_DRIVE_BR = 0,
    ROBOT_MOTOR_CHASSIS_DRIVE_BL,
    ROBOT_MOTOR_CHASSIS_DRIVE_FR,
    ROBOT_MOTOR_CHASSIS_DRIVE_FL,
    ROBOT_MOTOR_CHASSIS_STEER_BR,
    ROBOT_MOTOR_CHASSIS_STEER_BL,
    ROBOT_MOTOR_CHASSIS_STEER_FR,
    ROBOT_MOTOR_CHASSIS_STEER_FL,
    ROBOT_MOTOR_GIMBAL_BIG_YAW,
    ROBOT_MOTOR_GIMBAL_SMALL_YAW,
    ROBOT_MOTOR_GIMBAL_PITCH,
    ROBOT_MOTOR_SHOOT_LEFT,
    ROBOT_MOTOR_SHOOT_RIGHT,
    ROBOT_MOTOR_FEEDER,
    ROBOT_MOTOR_COUNT
} RobotActuator_MotorIdEnum;

typedef enum {
    ROBOT_GROUP_CHASSIS_DRIVE = 0,
    ROBOT_GROUP_CHASSIS_STEER,
    ROBOT_GROUP_GIMBAL_BIG_YAW,
    ROBOT_GROUP_GIMBAL_SMALL_YAW,
    ROBOT_GROUP_GIMBAL_PITCH,
    ROBOT_GROUP_SHOOTER,
    ROBOT_GROUP_COUNT
} RobotActuator_GroupIdEnum;

void RobotActuators_Init(void);
void RobotActuators_Service(void);
void RobotActuators_DecodeCan(CAN_HandleTypeDef *can,
                              uint32_t stdid,
                              uint8_t data[],
                              uint32_t len);
Actuator_MotorTypeDef *RobotActuators_GetMotor(RobotActuator_MotorIdEnum id);
Actuator_GroupTypeDef *RobotActuators_GetGroup(RobotActuator_GroupIdEnum id);
uint8_t RobotActuators_AnyOffline(void);

#ifdef __cplusplus
}
#endif

#endif
