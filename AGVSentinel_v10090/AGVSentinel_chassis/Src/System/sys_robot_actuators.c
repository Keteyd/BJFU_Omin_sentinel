#include "sys_robot_actuators.h"

#include "app_board_config.h"
#include "periph_DMmotor.h"
#include "periph_motor.h"
#include "sys_const.h"
#include <string.h>

typedef struct {
    Motor_MotorTypeDef *motor;
    uint8_t enabled;
} RobotActuator_DjiContextTypeDef;

typedef struct {
    DMmotor_t *motor;
    CAN_HandleTypeDef *can;
    uint32_t last_update_time;
    uint32_t last_enable_time;
    uint32_t offline_timeout_ms;
    float kp_set;
    float kd_set;
    float torque_set;
    float position_set;
    float velocity_set;
    uint8_t enabled;
} RobotActuator_DmContextTypeDef;

typedef struct {
    Motor_MotorGroupTypeDef *group;
} RobotActuator_DjiGroupContextTypeDef;

static Actuator_MotorTypeDef s_motors[ROBOT_MOTOR_COUNT];
static Actuator_GroupTypeDef s_groups[ROBOT_GROUP_COUNT];
static RobotActuator_DjiContextTypeDef s_dji_contexts[ROBOT_MOTOR_COUNT];
static RobotActuator_DmContextTypeDef s_pitch_context;
static RobotActuator_DjiGroupContextTypeDef s_group_contexts[ROBOT_GROUP_COUNT];

static int8_t RobotActuator_DjiEnable(void *context)
{
    RobotActuator_DjiContextTypeDef *driver = context;
    if (driver == NULL || driver->motor == NULL) return ACTUATOR_ERROR;
    driver->enabled = 1U;
    return ACTUATOR_OK;
}

static int8_t RobotActuator_DjiDisable(void *context)
{
    RobotActuator_DjiContextTypeDef *driver = context;
    if (driver == NULL || driver->motor == NULL) return ACTUATOR_ERROR;
    driver->enabled = 0U;
    Motor_SetMotorOutput(driver->motor, 0.0f);
    return ACTUATOR_OK;
}

static int8_t RobotActuator_DjiSetEffort(void *context, float effort)
{
    RobotActuator_DjiContextTypeDef *driver = context;
    if (driver == NULL || driver->motor == NULL) return ACTUATOR_ERROR;
    Motor_SetMotorOutput(driver->motor, driver->enabled ? effort : 0.0f);
    return ACTUATOR_OK;
}

static int8_t RobotActuator_UnsupportedCommand(void *context, float value)
{
    (void)context;
    (void)value;
    return ACTUATOR_UNSUPPORTED;
}

static int8_t RobotActuator_DjiFlush(void *context)
{
    (void)context;
    return ACTUATOR_OK;
}

static void RobotActuator_DjiReadFeedback(void *context, Actuator_FeedbackTypeDef *feedback)
{
    RobotActuator_DjiContextTypeDef *driver = context;
    if (driver == NULL || driver->motor == NULL || feedback == NULL) return;
    feedback->position = driver->motor->encoder.limited_angle;
    feedback->continuous_position = driver->motor->encoder.consequent_angle;
    feedback->velocity = driver->motor->encoder.speed;
    feedback->effort = driver->motor->encoder.current;
    feedback->temperature = driver->motor->encoder.temp;
    feedback->online = (uint8_t)!Motor_IsMotorOffline(driver->motor);
}

static const Actuator_MotorOpsTypeDef s_dji_ops = {
    RobotActuator_DjiEnable,
    RobotActuator_DjiDisable,
    RobotActuator_DjiSetEffort,
    RobotActuator_UnsupportedCommand,
    RobotActuator_UnsupportedCommand,
    RobotActuator_DjiFlush,
    RobotActuator_DjiReadFeedback
};

static int8_t RobotActuator_DmEnable(void *context)
{
    RobotActuator_DmContextTypeDef *driver = context;
    if (driver == NULL || driver->motor == NULL || driver->can == NULL) return ACTUATOR_ERROR;
    driver->motor->ctrl.kp_set = driver->kp_set;
    driver->motor->ctrl.kd_set = driver->kd_set;
    driver->motor->ctrl.tor_set = driver->torque_set;
    driver->motor->ctrl.pos_set = driver->position_set;
    driver->motor->ctrl.vel_set = driver->velocity_set;
    dm_motor_enable(driver->can, driver->motor);
    driver->last_enable_time = HAL_GetTick();
    driver->enabled = 1U;
    return ACTUATOR_OK;
}

static int8_t RobotActuator_DmDisable(void *context)
{
    RobotActuator_DmContextTypeDef *driver = context;
    if (driver == NULL || driver->motor == NULL || driver->can == NULL) return ACTUATOR_ERROR;
    dm_motor_disable(driver->can, driver->motor);
    driver->enabled = 0U;
    return ACTUATOR_OK;
}

static int8_t RobotActuator_DmSetEffort(void *context, float effort)
{
    RobotActuator_DmContextTypeDef *driver = context;
    if (driver == NULL || driver->motor == NULL) return ACTUATOR_ERROR;
    driver->torque_set = effort;
    driver->motor->ctrl.tor_set = effort;
    return ACTUATOR_OK;
}

static int8_t RobotActuator_DmSetVelocity(void *context, float velocity)
{
    RobotActuator_DmContextTypeDef *driver = context;
    if (driver == NULL || driver->motor == NULL) return ACTUATOR_ERROR;
    driver->velocity_set = velocity;
    driver->motor->ctrl.vel_set = velocity;
    return ACTUATOR_OK;
}

static int8_t RobotActuator_DmSetPosition(void *context, float position)
{
    RobotActuator_DmContextTypeDef *driver = context;
    if (driver == NULL || driver->motor == NULL) return ACTUATOR_ERROR;
    driver->position_set = position;
    driver->motor->ctrl.pos_set = position;
    return ACTUATOR_OK;
}

static int8_t RobotActuator_DmFlush(void *context)
{
    RobotActuator_DmContextTypeDef *driver = context;
    if (driver == NULL || driver->motor == NULL || driver->can == NULL) return ACTUATOR_ERROR;
    if (driver->enabled) dm_motor_ctrl_send(driver->can, driver->motor);
    return ACTUATOR_OK;
}

static void RobotActuator_DmReadFeedback(void *context, Actuator_FeedbackTypeDef *feedback)
{
    RobotActuator_DmContextTypeDef *driver = context;
    if (driver == NULL || driver->motor == NULL || feedback == NULL) return;
    feedback->position = driver->motor->para.pos;
    feedback->continuous_position = driver->motor->para.pos;
    feedback->velocity = driver->motor->para.vel;
    feedback->effort = driver->motor->para.tor;
    feedback->temperature = driver->motor->para.Tmos;
    feedback->online = (uint8_t)(driver->last_update_time != 0U &&
                                 (HAL_GetTick() - driver->last_update_time) <= driver->offline_timeout_ms);
}

static const Actuator_MotorOpsTypeDef s_dm_ops = {
    RobotActuator_DmEnable,
    RobotActuator_DmDisable,
    RobotActuator_DmSetEffort,
    RobotActuator_DmSetVelocity,
    RobotActuator_DmSetPosition,
    RobotActuator_DmFlush,
    RobotActuator_DmReadFeedback
};

static int8_t RobotActuator_DjiGroupFlush(void *context)
{
    RobotActuator_DjiGroupContextTypeDef *driver = context;
    if (driver == NULL || driver->group == NULL) return ACTUATOR_ERROR;
    Motor_SendMotorGroupOutput(driver->group);
    return ACTUATOR_OK;
}

static int8_t RobotActuator_PitchGroupFlush(void *context)
{
    return Actuator_MotorFlush((Actuator_MotorTypeDef *)context);
}

static void RobotActuator_BindDji(RobotActuator_MotorIdEnum id,
                                  const char *name,
                                  Motor_MotorTypeDef *legacy_motor)
{
    s_dji_contexts[id].motor = legacy_motor;
    s_dji_contexts[id].enabled = 1U;
    Actuator_MotorBind(&s_motors[id], name, &s_dji_ops, &s_dji_contexts[id]);
}

static void RobotActuator_BindDjiGroup(RobotActuator_GroupIdEnum id,
                                       const char *name,
                                       Motor_MotorGroupTypeDef *legacy_group)
{
    s_group_contexts[id].group = legacy_group;
    s_groups[id].name = name;
    s_groups[id].flush = RobotActuator_DjiGroupFlush;
    s_groups[id].context = &s_group_contexts[id];
}

void RobotActuators_Init(void)
{
    memset(s_motors, 0, sizeof(s_motors));
    memset(s_groups, 0, sizeof(s_groups));
    memset(s_dji_contexts, 0, sizeof(s_dji_contexts));
    memset(s_group_contexts, 0, sizeof(s_group_contexts));
    memset(&s_pitch_context, 0, sizeof(s_pitch_context));

    Motor_InitAllMotors();

#if BOARD_HAS_CHASSIS
    RobotActuator_BindDji(ROBOT_MOTOR_CHASSIS_DRIVE_BR, "chassis.drive.br", &Motor_ChassisBackRightMotor);
    RobotActuator_BindDji(ROBOT_MOTOR_CHASSIS_DRIVE_BL, "chassis.drive.bl", &Motor_ChassisBackLeftMotor);
    RobotActuator_BindDji(ROBOT_MOTOR_CHASSIS_DRIVE_FR, "chassis.drive.fr", &Motor_ChassisFontRightMotor);
    RobotActuator_BindDji(ROBOT_MOTOR_CHASSIS_DRIVE_FL, "chassis.drive.fl", &Motor_ChassisFontLeftMotor);
    RobotActuator_BindDji(ROBOT_MOTOR_CHASSIS_STEER_BR, "chassis.steer.br", &Motor_ChassisBackRightSteerMotor);
    RobotActuator_BindDji(ROBOT_MOTOR_CHASSIS_STEER_BL, "chassis.steer.bl", &Motor_ChassisBackLeftSteerMotor);
    RobotActuator_BindDji(ROBOT_MOTOR_CHASSIS_STEER_FR, "chassis.steer.fr", &Motor_ChassisFontRightSteerMotor);
    RobotActuator_BindDji(ROBOT_MOTOR_CHASSIS_STEER_FL, "chassis.steer.fl", &Motor_ChassisFontLeftSteerMotor);
    RobotActuator_BindDjiGroup(ROBOT_GROUP_CHASSIS_DRIVE, "chassis.drive", &Motor_ChassisMotors);
    RobotActuator_BindDjiGroup(ROBOT_GROUP_CHASSIS_STEER, "chassis.steer", &Motor_ChassisSteerMotors);
#endif

#if BOARD_HAS_GIMBAL
    RobotActuator_BindDji(ROBOT_MOTOR_GIMBAL_BIG_YAW, "gimbal.big_yaw", &Motor_Big_YawMotor);
    RobotActuator_BindDji(ROBOT_MOTOR_GIMBAL_SMALL_YAW, "gimbal.small_yaw", &Motor_Small_YawMotor);
    RobotActuator_BindDji(ROBOT_MOTOR_SHOOT_LEFT, "shooter.left", &Motor_ShootLeftMotor);
    RobotActuator_BindDji(ROBOT_MOTOR_SHOOT_RIGHT, "shooter.right", &Motor_ShootRightMotor);
    RobotActuator_BindDji(ROBOT_MOTOR_FEEDER, "shooter.feeder", &Motor_FeedMotor);
    RobotActuator_BindDjiGroup(ROBOT_GROUP_GIMBAL_BIG_YAW, "gimbal.big_yaw", &Motor_Big_YawMotors);
    RobotActuator_BindDjiGroup(ROBOT_GROUP_GIMBAL_SMALL_YAW, "gimbal.small_yaw", &Motor_Small_YawMotors);
    RobotActuator_BindDjiGroup(ROBOT_GROUP_SHOOTER, "shooter", &Motor_ShootMotors);

    dm_motor_init();
    motor[Motor1].id = Const_DM4310_PitchMotorId;
    motor[Motor1].mst_id = Const_DM4310_PitchFeedbackId;
    s_pitch_context.motor = &motor[Motor1];
    s_pitch_context.can = &hcan2;
    s_pitch_context.offline_timeout_ms = Const_DM4310_OfflineMs;
    s_pitch_context.kp_set = motor[Motor1].ctrl.kp_set;
    s_pitch_context.kd_set = motor[Motor1].ctrl.kd_set;
    s_pitch_context.torque_set = motor[Motor1].ctrl.tor_set;
    s_pitch_context.position_set = motor[Motor1].ctrl.pos_set;
    s_pitch_context.velocity_set = motor[Motor1].ctrl.vel_set;
    Actuator_MotorBind(&s_motors[ROBOT_MOTOR_GIMBAL_PITCH],
                       "gimbal.pitch", &s_dm_ops, &s_pitch_context);
    s_groups[ROBOT_GROUP_GIMBAL_PITCH].name = "gimbal.pitch";
    s_groups[ROBOT_GROUP_GIMBAL_PITCH].flush = RobotActuator_PitchGroupFlush;
    s_groups[ROBOT_GROUP_GIMBAL_PITCH].context = &s_motors[ROBOT_MOTOR_GIMBAL_PITCH];
    (void)Actuator_MotorEnable(&s_motors[ROBOT_MOTOR_GIMBAL_PITCH]);
#endif
}

void RobotActuators_Service(void)
{
#if BOARD_HAS_GIMBAL
    uint32_t now = HAL_GetTick();
    Actuator_FeedbackTypeDef feedback = Actuator_MotorGetFeedback(&s_motors[ROBOT_MOTOR_GIMBAL_PITCH]);
    if (s_pitch_context.enabled && !feedback.online &&
        (now - s_pitch_context.last_enable_time) >= 100U) {
        (void)Actuator_MotorEnable(&s_motors[ROBOT_MOTOR_GIMBAL_PITCH]);
    }
#endif
}

void RobotActuators_DecodeCan(CAN_HandleTypeDef *can,
                              uint32_t stdid,
                              uint8_t data[],
                              uint32_t len)
{
    Motor_EncoderDecodeCallback(can, stdid, data, len);
#if BOARD_HAS_GIMBAL
    if (can == s_pitch_context.can && s_pitch_context.motor != NULL &&
        stdid == s_pitch_context.motor->mst_id && len == 8U) {
        dm_motor_fbdata(s_pitch_context.motor, data);
        s_pitch_context.last_update_time = HAL_GetTick();
    }
#endif
}

Actuator_MotorTypeDef *RobotActuators_GetMotor(RobotActuator_MotorIdEnum id)
{
    if ((uint32_t)id >= ROBOT_MOTOR_COUNT || s_motors[id].ops == NULL) return NULL;
    return &s_motors[id];
}

Actuator_GroupTypeDef *RobotActuators_GetGroup(RobotActuator_GroupIdEnum id)
{
    if ((uint32_t)id >= ROBOT_GROUP_COUNT || s_groups[id].flush == NULL) return NULL;
    return &s_groups[id];
}

uint8_t RobotActuators_AnyOffline(void)
{
    uint32_t id;
    for (id = 0U; id < ROBOT_MOTOR_COUNT; ++id) {
        if (s_motors[id].ops != NULL && !Actuator_MotorIsOnline(&s_motors[id])) return 1U;
    }
    return 0U;
}
