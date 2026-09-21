#include "module_gimbal_axis.h"

#include <string.h>

void GimbalAxis_Init(GimbalAxis_TypeDef *axis,
                     Actuator_MotorTypeDef *motor,
                     Actuator_GroupTypeDef *group,
                     float zero_position)
{
    if (axis == NULL) return;
    memset(axis, 0, sizeof(*axis));
    axis->motor = motor;
    axis->group = group;
    axis->zero_position = zero_position;
    axis->enabled = 1U;
}

void GimbalAxis_SetEnabled(GimbalAxis_TypeDef *axis, uint8_t enabled)
{
    if (axis == NULL || axis->motor == NULL) return;
    enabled = enabled != 0U;
    if (axis->enabled == enabled) return;
    if (enabled) {
        (void)Actuator_MotorEnable(axis->motor);
    } else {
        (void)Actuator_MotorSetEffort(axis->motor, 0.0f);
        (void)Actuator_MotorDisable(axis->motor);
    }
    axis->enabled = enabled;
}

void GimbalAxis_SetEffort(GimbalAxis_TypeDef *axis, float effort)
{
    if (axis == NULL || axis->motor == NULL) return;
    (void)Actuator_MotorSetEffort(axis->motor, axis->enabled ? effort : 0.0f);
}

void GimbalAxis_SetPosition(GimbalAxis_TypeDef *axis, float position)
{
    if (axis == NULL || axis->motor == NULL) return;
    (void)Actuator_MotorSetPosition(axis->motor, position);
}

void GimbalAxis_Flush(GimbalAxis_TypeDef *axis)
{
    if (axis == NULL) return;
    if (axis->group != NULL) {
        (void)Actuator_GroupFlush(axis->group);
    } else {
        (void)Actuator_MotorFlush(axis->motor);
    }
}

Actuator_FeedbackTypeDef GimbalAxis_GetFeedback(GimbalAxis_TypeDef *axis)
{
    if (axis == NULL) {
        Actuator_FeedbackTypeDef feedback;
        memset(&feedback, 0, sizeof(feedback));
        return feedback;
    }
    return Actuator_MotorGetFeedback(axis->motor);
}

float GimbalAxis_GetLogicalPosition(GimbalAxis_TypeDef *axis)
{
    return GimbalAxis_GetFeedback(axis).position - (axis != NULL ? axis->zero_position : 0.0f);
}
