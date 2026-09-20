#include <assert.h>
#include <stdio.h>
#include "module_small_yaw_rate.h"
#include "module_yaw_limits.h"

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

int main(void)
{
    SmallYawRate_Command command = {0};
    assert(SMALL_YAW_ANGLE_OPEN_TEST == 0U);
    near(SmallYawRate_GetRpm(&command, 0), 0);
    SmallYawRate_SetDps(&command, 132, 100);
    near(SmallYawRate_GetRpm(&command, 100), 22);
    near(SmallYawRate_GetRpm(&command, 120), 22);
    near(SmallYawRate_GetRpm(&command, 121), 0);
    assert(!command.valid);
    /* A timeout remains cleared even if the tick counter later wraps. */
    near(SmallYawRate_GetRpm(&command, 100), 0);
    SmallYawRate_SetDps(&command, -132, 200);
    near(SmallYawRate_GetRpm(&command, 200), -22);
    SmallYawRate_SetDps(&command, 0, 202);
    near(SmallYawRate_GetRpm(&command, 202), 0);
    SmallYawRate_SetDps(&command, 60, UINT32_MAX - 5U);
    near(SmallYawRate_GetRpm(&command, 4), 10);
    near(SmallYawRate_GetRpm(&command, 15), 0);
    SmallYawRate_SetDps(&command, 60, 300);
    SmallYawRate_SetDps(&command, NAN, 301);
    near(SmallYawRate_GetRpm(&command, 301), 0);
    SmallYawRate_SetDps(&command, INFINITY, 302);
    near(SmallYawRate_GetRpm(&command, 302), 0);
    SmallYawRate_SetDps(&command, -INFINITY, 303);
    near(SmallYawRate_GetRpm(&command, 303), 0);
    SmallYawRate_SetDps(&command, 60, 400);
    SmallYawRate_Reset(&command);
    near(SmallYawRate_GetRpm(&command, 400), 0);

    /* The rate-only target still passes through the existing joint guards. */
    SmallYawRate_SetDps(&command, 60, 500);
    near(YawLimits_Speed(0, SmallYawRate_GetRpm(&command, 500),
        0, 0, 1, YAW_RATE_UNLIMITED), 10);
    near(YawLimits_Speed(YAW_LIMIT_MAX_DEG,
        SmallYawRate_GetRpm(&command, 500), 0, 0, 1, YAW_RATE_UNLIMITED), 0);
    SmallYawRate_SetDps(&command, -60, 502);
    near(YawLimits_Speed(YAW_LIMIT_MIN_DEG,
        SmallYawRate_GetRpm(&command, 502), 0, 0, 1, YAW_RATE_UNLIMITED), 0);
    near(YawLimits_Speed(YAW_LIMIT_MAX_DEG,
        SmallYawRate_GetRpm(&command, 502), 0, 0, 1, YAW_RATE_UNLIMITED), -10);
    /* At a stop, compensate base motion without requesting outward joint motion. */
    near(YawLimits_Speed(YAW_LIMIT_MAX_DEG, 0, -5, 0,
        1, YAW_RATE_UNLIMITED), -5);
    near(YawLimits_Effort(YAW_LIMIT_MAX_DEG, 6), 0);
    near(YawLimits_Effort(YAW_LIMIT_MIN_DEG, -6), 0);
    puts("PASS: small-yaw rate units, release, timeout, wrap, reset and soft limits");
    return 0;
}
