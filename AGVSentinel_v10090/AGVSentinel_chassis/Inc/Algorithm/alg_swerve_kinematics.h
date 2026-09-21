#ifndef ALG_SWERVE_KINEMATICS_H
#define ALG_SWERVE_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float angle_deg;
    float speed;
} SwerveKinematics_ModuleStateTypeDef;

float SwerveKinematics_Normalize360(float angle_deg);
float SwerveKinematics_WrapError(float error_deg);
void SwerveKinematics_Solve(float vx,
                            float vy,
                            float wz,
                            float half_length,
                            float half_width,
                            const float current_angle_deg[4],
                            SwerveKinematics_ModuleStateTypeDef output[4]);

#ifdef __cplusplus
}
#endif

#endif
