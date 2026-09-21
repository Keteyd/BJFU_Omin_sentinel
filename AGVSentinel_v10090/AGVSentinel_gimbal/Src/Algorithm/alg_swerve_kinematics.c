#include "alg_swerve_kinematics.h"

#include <math.h>
#include <stddef.h>

#define SWERVE_PI 3.14159265358979323846f

float SwerveKinematics_Normalize360(float angle_deg)
{
    while (angle_deg >= 360.0f) angle_deg -= 360.0f;
    while (angle_deg < 0.0f) angle_deg += 360.0f;
    return angle_deg;
}

float SwerveKinematics_WrapError(float error_deg)
{
    while (error_deg > 180.0f) error_deg -= 360.0f;
    while (error_deg < -180.0f) error_deg += 360.0f;
    return error_deg;
}

static void SwerveKinematics_SolveOne(float wheel_x,
                                      float wheel_y,
                                      float vx,
                                      float vy,
                                      float wz,
                                      float current_angle_deg,
                                      SwerveKinematics_ModuleStateTypeDef *output)
{
    float wheel_vx = vx + wz * wheel_y;
    float wheel_vy = vy - wz * wheel_x;
    float speed = sqrtf(wheel_vx * wheel_vx + wheel_vy * wheel_vy);
    float target_angle;
    float error;

    if (speed <= 1.0e-4f) {
        output->angle_deg = SwerveKinematics_Normalize360(current_angle_deg);
        output->speed = 0.0f;
        return;
    }

    target_angle = SwerveKinematics_Normalize360(
        atan2f(wheel_vy, wheel_vx) * 180.0f / SWERVE_PI);
    error = SwerveKinematics_WrapError(target_angle - current_angle_deg);
    if (error > 90.0f) {
        target_angle = SwerveKinematics_Normalize360(target_angle - 180.0f);
        speed = -speed;
    } else if (error < -90.0f) {
        target_angle = SwerveKinematics_Normalize360(target_angle + 180.0f);
        speed = -speed;
    }

    output->angle_deg = target_angle;
    output->speed = speed;
}

void SwerveKinematics_Solve(float vx,
                            float vy,
                            float wz,
                            float half_length,
                            float half_width,
                            const float current_angle_deg[4],
                            SwerveKinematics_ModuleStateTypeDef output[4])
{
    if (current_angle_deg == NULL || output == NULL) return;
    SwerveKinematics_SolveOne(-half_length, half_width, vx, vy, wz,
                              current_angle_deg[0], &output[0]);
    SwerveKinematics_SolveOne(-half_length, -half_width, vx, vy, wz,
                              current_angle_deg[1], &output[1]);
    SwerveKinematics_SolveOne(half_length, half_width, vx, vy, wz,
                              current_angle_deg[2], &output[2]);
    SwerveKinematics_SolveOne(half_length, -half_width, vx, vy, wz,
                              current_angle_deg[3], &output[3]);
}
