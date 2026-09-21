#include "lib_actuator.h"

#include <string.h>

void Actuator_MotorBind(Actuator_MotorTypeDef *motor,
                        const char *name,
                        const Actuator_MotorOpsTypeDef *ops,
                        void *context)
{
    if (motor == NULL) return;
    motor->name = name;
    motor->ops = ops;
    motor->context = context;
}

int8_t Actuator_MotorEnable(Actuator_MotorTypeDef *motor)
{
    if (motor == NULL || motor->ops == NULL || motor->ops->enable == NULL) return ACTUATOR_ERROR;
    return motor->ops->enable(motor->context);
}

int8_t Actuator_MotorDisable(Actuator_MotorTypeDef *motor)
{
    if (motor == NULL || motor->ops == NULL || motor->ops->disable == NULL) return ACTUATOR_ERROR;
    return motor->ops->disable(motor->context);
}

int8_t Actuator_MotorSetEffort(Actuator_MotorTypeDef *motor, float effort)
{
    if (motor == NULL || motor->ops == NULL || motor->ops->set_effort == NULL) return ACTUATOR_UNSUPPORTED;
    return motor->ops->set_effort(motor->context, effort);
}

int8_t Actuator_MotorSetVelocity(Actuator_MotorTypeDef *motor, float velocity)
{
    if (motor == NULL || motor->ops == NULL || motor->ops->set_velocity == NULL) return ACTUATOR_UNSUPPORTED;
    return motor->ops->set_velocity(motor->context, velocity);
}

int8_t Actuator_MotorSetPosition(Actuator_MotorTypeDef *motor, float position)
{
    if (motor == NULL || motor->ops == NULL || motor->ops->set_position == NULL) return ACTUATOR_UNSUPPORTED;
    return motor->ops->set_position(motor->context, position);
}

int8_t Actuator_MotorFlush(Actuator_MotorTypeDef *motor)
{
    if (motor == NULL || motor->ops == NULL || motor->ops->flush == NULL) return ACTUATOR_UNSUPPORTED;
    return motor->ops->flush(motor->context);
}

Actuator_FeedbackTypeDef Actuator_MotorGetFeedback(Actuator_MotorTypeDef *motor)
{
    Actuator_FeedbackTypeDef feedback;
    memset(&feedback, 0, sizeof(feedback));
    if (motor != NULL && motor->ops != NULL && motor->ops->read_feedback != NULL) {
        motor->ops->read_feedback(motor->context, &feedback);
    }
    return feedback;
}

uint8_t Actuator_MotorIsOnline(Actuator_MotorTypeDef *motor)
{
    return Actuator_MotorGetFeedback(motor).online;
}

int8_t Actuator_GroupFlush(Actuator_GroupTypeDef *group)
{
    if (group == NULL || group->flush == NULL) return ACTUATOR_ERROR;
    return group->flush(group->context);
}
