#ifndef LIB_ACTUATOR_H
#define LIB_ACTUATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    ACTUATOR_OK = 0,
    ACTUATOR_ERROR = -1,
    ACTUATOR_UNSUPPORTED = -2
} Actuator_ResultEnum;

typedef struct {
    float position;
    float continuous_position;
    float velocity;
    float effort;
    float temperature;
    uint8_t online;
} Actuator_FeedbackTypeDef;

struct ActuatorMotor;

typedef struct {
    int8_t (*enable)(void *context);
    int8_t (*disable)(void *context);
    int8_t (*set_effort)(void *context, float effort);
    int8_t (*set_velocity)(void *context, float velocity);
    int8_t (*set_position)(void *context, float position);
    int8_t (*flush)(void *context);
    void (*read_feedback)(void *context, Actuator_FeedbackTypeDef *feedback);
} Actuator_MotorOpsTypeDef;

typedef struct ActuatorMotor {
    const char *name;
    const Actuator_MotorOpsTypeDef *ops;
    void *context;
} Actuator_MotorTypeDef;

typedef struct {
    const char *name;
    int8_t (*flush)(void *context);
    void *context;
} Actuator_GroupTypeDef;

void Actuator_MotorBind(Actuator_MotorTypeDef *motor,
                        const char *name,
                        const Actuator_MotorOpsTypeDef *ops,
                        void *context);
int8_t Actuator_MotorEnable(Actuator_MotorTypeDef *motor);
int8_t Actuator_MotorDisable(Actuator_MotorTypeDef *motor);
int8_t Actuator_MotorSetEffort(Actuator_MotorTypeDef *motor, float effort);
int8_t Actuator_MotorSetVelocity(Actuator_MotorTypeDef *motor, float velocity);
int8_t Actuator_MotorSetPosition(Actuator_MotorTypeDef *motor, float position);
int8_t Actuator_MotorFlush(Actuator_MotorTypeDef *motor);
Actuator_FeedbackTypeDef Actuator_MotorGetFeedback(Actuator_MotorTypeDef *motor);
uint8_t Actuator_MotorIsOnline(Actuator_MotorTypeDef *motor);
int8_t Actuator_GroupFlush(Actuator_GroupTypeDef *group);

#ifdef __cplusplus
}
#endif

#endif
