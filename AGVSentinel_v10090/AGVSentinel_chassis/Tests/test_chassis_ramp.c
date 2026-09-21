#include "alg_chassis_ramp.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TEST_EPSILON (1.0e-4f)

static const ChassisRamp_ConfigTypeDef kConfig = {
    10.0f, 20.0f, 50.0f, 30.0f, 60.0f, 90.0f, 0.020f
};

static void ExpectNear(float actual, float expected)
{
    assert(fabsf(actual - expected) < TEST_EPSILON);
}

static void TestTranslationPreservesDirection(void)
{
    ChassisRamp_StateTypeDef state = {0};
    ChassisRamp_Step(&state, 3.0f, 4.0f, 0.0f, 0.1f, &kConfig);
    /* dt is capped to 20 ms: vector step length is 10 * .02 = .2. */
    ExpectNear(state.vx, 0.12f);
    ExpectNear(state.vy, 0.16f);
}

static void TestReleaseUsesFasterDeceleration(void)
{
    ChassisRamp_StateTypeDef state = {1.0f, 0.0f, 0.0f};
    ChassisRamp_Step(&state, 0.0f, 0.0f, 0.0f, 0.02f, &kConfig);
    ExpectNear(state.vx, 0.0f);
    ExpectNear(state.vy, 0.0f);
}

static void TestNonzeroSlowdownKeepsNormalDeceleration(void)
{
    ChassisRamp_StateTypeDef state = {1.0f, 0.0f, 0.0f};
    ChassisRamp_Step(&state, 0.2f, 0.0f, 0.0f, 0.02f, &kConfig);
    ExpectNear(state.vx, 0.6f);
}

static void TestDirectionChangeIsVectorLimited(void)
{
    ChassisRamp_StateTypeDef state = {1.0f, 0.0f, 0.0f};
    float step_length;
    ChassisRamp_Step(&state, 0.0f, 1.0f, 0.0f, 0.02f, &kConfig);
    step_length = sqrtf((state.vx - 1.0f) * (state.vx - 1.0f) +
                        state.vy * state.vy);
    ExpectNear(step_length, 0.4f);
    assert(state.vx < 1.0f && state.vy > 0.0f);
}

static void TestYawAccelerationAndBraking(void)
{
    ChassisRamp_StateTypeDef state = {0};
    ChassisRamp_Step(&state, 0.0f, 0.0f, 100.0f, 0.02f, &kConfig);
    ExpectNear(state.wz, 0.6f);
    ChassisRamp_Step(&state, 0.0f, 0.0f, 0.0f, 0.02f, &kConfig);
    ExpectNear(state.wz, 0.0f);
}

static void TestResetIsImmediate(void)
{
    ChassisRamp_StateTypeDef state = {3.0f, -2.0f, 1.0f};
    ChassisRamp_Reset(&state);
    ExpectNear(state.vx, 0.0f);
    ExpectNear(state.vy, 0.0f);
    ExpectNear(state.wz, 0.0f);
}

int main(void)
{
    TestTranslationPreservesDirection();
    TestReleaseUsesFasterDeceleration();
    TestNonzeroSlowdownKeepsNormalDeceleration();
    TestDirectionChangeIsVectorLimited();
    TestYawAccelerationAndBraking();
    TestResetIsImmediate();
    puts("chassis ramp tests passed");
    return 0;
}
