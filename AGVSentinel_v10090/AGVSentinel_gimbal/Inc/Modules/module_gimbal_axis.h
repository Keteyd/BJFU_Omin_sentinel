#ifndef MODULE_GIMBAL_AXIS_H
#define MODULE_GIMBAL_AXIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lib_actuator.h"
#include <stdint.h>

typedef struct {
    Actuator_MotorTypeDef *motor;
    Actuator_GroupTypeDef *group;
    float zero_position;
    uint8_t enabled;
} GimbalAxis_TypeDef;

void GimbalAxis_Init(GimbalAxis_TypeDef *axis,
                     Actuator_MotorTypeDef *motor,
                     Actuator_GroupTypeDef *group,
                     float zero_position);
void GimbalAxis_SetEnabled(GimbalAxis_TypeDef *axis, uint8_t enabled);
void GimbalAxis_SetEffort(GimbalAxis_TypeDef *axis, float effort);
void GimbalAxis_SetPosition(GimbalAxis_TypeDef *axis, float position);
void GimbalAxis_Flush(GimbalAxis_TypeDef *axis);
Actuator_FeedbackTypeDef GimbalAxis_GetFeedback(GimbalAxis_TypeDef *axis);
float GimbalAxis_GetLogicalPosition(GimbalAxis_TypeDef *axis);

#ifdef __cplusplus
}
#endif

#endif
