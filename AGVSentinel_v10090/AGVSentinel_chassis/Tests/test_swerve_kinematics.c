#include "alg_swerve_kinematics.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TEST_EPSILON (1.0e-4f)

static void ExpectNear(float actual, float expected)
{
    assert(fabsf(actual - expected) < TEST_EPSILON);
}

static void TestForwardAndStrafe(void)
{
    const float current[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    SwerveKinematics_ModuleStateTypeDef output[4];
    unsigned int i;

    SwerveKinematics_Solve(1.0f, 0.0f, 0.0f, 1.0f, 1.0f, current, output);
    for (i = 0U; i < 4U; ++i) {
        ExpectNear(output[i].angle_deg, 0.0f);
        ExpectNear(output[i].speed, 1.0f);
    }

    SwerveKinematics_Solve(0.0f, 1.0f, 0.0f, 1.0f, 1.0f, current, output);
    for (i = 0U; i < 4U; ++i) {
        ExpectNear(output[i].angle_deg, 90.0f);
        ExpectNear(output[i].speed, 1.0f);
    }
}

static void TestReverseUsesWheelFlip(void)
{
    const float current[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    SwerveKinematics_ModuleStateTypeDef output[4];
    unsigned int i;

    SwerveKinematics_Solve(-1.0f, 0.0f, 0.0f, 1.0f, 1.0f, current, output);
    for (i = 0U; i < 4U; ++i) {
        ExpectNear(output[i].angle_deg, 0.0f);
        ExpectNear(output[i].speed, -1.0f);
    }
}

static void TestRotationGeometry(void)
{
    const float current[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    const float speed = 1.41421356237f;
    SwerveKinematics_ModuleStateTypeDef output[4];

    SwerveKinematics_Solve(0.0f, 0.0f, 1.0f, 1.0f, 1.0f, current, output);
    ExpectNear(output[0].angle_deg, 45.0f);
    ExpectNear(output[0].speed, speed);
    ExpectNear(output[1].angle_deg, 315.0f);
    ExpectNear(output[1].speed, -speed);
    ExpectNear(output[2].angle_deg, 315.0f);
    ExpectNear(output[2].speed, speed);
    ExpectNear(output[3].angle_deg, 45.0f);
    ExpectNear(output[3].speed, -speed);
}

int main(void)
{
    TestForwardAndStrafe();
    TestReverseUsesWheelFlip();
    TestRotationGeometry();
    puts("swerve kinematics tests passed");
    return 0;
}
