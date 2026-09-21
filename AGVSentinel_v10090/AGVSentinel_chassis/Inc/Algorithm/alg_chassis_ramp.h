#ifndef ALG_CHASSIS_RAMP_H
#define ALG_CHASSIS_RAMP_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float translation_accel_per_s;
    float translation_decel_per_s;
    float translation_stop_per_s;
    float yaw_accel_per_s;
    float yaw_decel_per_s;
    float yaw_stop_per_s;
    float maximum_dt_s;
} ChassisRamp_ConfigTypeDef;

typedef struct {
    float vx;
    float vy;
    float wz;
} ChassisRamp_StateTypeDef;

void ChassisRamp_Reset(ChassisRamp_StateTypeDef *state);
void ChassisRamp_Step(ChassisRamp_StateTypeDef *state,
                      float target_vx,
                      float target_vy,
                      float target_wz,
                      float dt_s,
                      const ChassisRamp_ConfigTypeDef *config);

#ifdef __cplusplus
}
#endif

#endif
