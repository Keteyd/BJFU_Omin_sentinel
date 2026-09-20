#include <assert.h>
#include <stdio.h>
#include "module_yaw_limits.h"

static void near(float a, float b)
{
    assert(fabsf(a - b) < 0.0001f);
}

int main(void)
{
    float joint;
    float lo = YAW_LIMIT_MIN_DEG, hi = YAW_LIMIT_MAX_DEG;
    int sign;
    assert(YawLimits_RateCap(YAW_RATE_UNLIMITED) == FLT_MAX);
    near(YawLimits_RateCap(0), 0);
    near(YawLimits_RateCap(24), 24);
    near(YawLimits_RateCap(-0.5f), 0);
    near(YawLimits_RateCap(NAN), 0);
    near(YawLimits_RateCap(INFINITY), 0);
    near(YawLimits_DecodeRateCap(UINT16_MAX, 0.01f), YAW_RATE_UNLIMITED);
    near(YawLimits_DecodeRateCap(UINT16_MAX, 0.0001f), YAW_RATE_UNLIMITED);
    near(YawLimits_DecodeRateCap(0, 0.01f), 0);
    near(YawLimits_DecodeRateCap(2400, 0.01f), 24);
    near(YawLimits_DecodeRateCap(3000, 0.0001f), 0.3f);
    near(YAW_LIMIT_ZERO_DEG, 299.53125f);
    near(lo, -56.677734375f);
    near(hi, 29.080078125f);
    assert(YawLimits_Position(YAW_LIMIT_ZERO_DEG, &joint));
    near(joint, 0);
    assert(YawLimits_Position(5458.0f * 360 / 8192, &joint));
    near(joint, YAW_LIMIT_HARD_MIN_DEG);
    assert(YawLimits_Position(7546.0f * 360 / 8192, &joint));
    near(joint, YAW_LIMIT_HARD_MAX_DEG);
    assert(!YawLimits_Position(0, &joint));
    assert(!YawLimits_Position(360, &joint));
    assert(!YawLimits_Position(NAN, &joint));
    assert(!YawLimits_Position(INFINITY, &joint));
    for (sign = -1; sign <= 1; sign += 2) {
        near(YawLimits_Error(0, 5, (float)sign), 5);
        near(YawLimits_Error(hi, sign * 20, (float)sign), 0);
        near(YawLimits_Error(lo, sign * -20, (float)sign), 0);
        near(YawLimits_Error(hi, sign * -2, (float)sign), sign * -2);
        near(YawLimits_Error(lo, sign * 2, (float)sign), sign * 2);
        /* Outside soft bounds, no automatic return; inward input still works. */
        near(YawLimits_Error(hi + 2, 0, (float)sign), 0);
        near(YawLimits_Error(hi + 2, sign * -1, (float)sign), sign * -1);
        near(YawLimits_Error(lo - 2, 0, (float)sign), 0);
        near(YawLimits_Error(lo - 2, sign * 1, (float)sign), sign * 1);
        near(YawLimits_Speed(0, 10, 0, 0, (float)sign, 24), 10);
        near(YawLimits_Speed(hi, sign * 10, 0, 0, (float)sign, 24), 0);
        near(YawLimits_Speed(lo, sign * -10, 0, 0, (float)sign, 24), 0);
        near(YawLimits_Speed(hi - 4, sign * 24, 0, 0, (float)sign, 24), sign * 12);
        near(YawLimits_Speed(lo + 4, sign * -24, 0, 0, (float)sign, 24), sign * -12);
        near(YawLimits_Speed(hi, sign * -10, 0, 0, (float)sign, 24), sign * -10);
        /* World speed is zero, but base motion would drive the joint outward. */
        near(YawLimits_Speed(hi, 0, 0, 5, (float)sign, 24), sign * -5);
        near(YawLimits_Speed(lo, 0, 0, -5, (float)sign, 24), sign * 5);
        /* Uncapped cruising, but unchanged braking slope at either endpoint. */
        near(YawLimits_Speed(0, sign * 60, 0, 0, (float)sign, -1), sign * 60);
        near(YawLimits_Speed(0, sign * -60, 0, 0, (float)sign, -1), sign * -60);
        near(YawLimits_Speed(hi - 4, sign * 120, 0, 0, (float)sign, -1), sign * 12);
        near(YawLimits_Speed(lo + 4, sign * -120, 0, 0, (float)sign, -1), sign * -12);
        near(YawLimits_Speed(hi, sign * 120, 0, 0, (float)sign, -1), 0);
        near(YawLimits_Speed(lo, sign * -120, 0, 0, (float)sign, -1), 0);
        near(YawLimits_Speed(hi, sign * -60, 0, 0, (float)sign, -1), sign * -60);
        near(YawLimits_Speed(lo, sign * 60, 0, 0, (float)sign, -1), sign * 60);
        near(YawLimits_Speed(hi, 0, 0, 5, (float)sign, -1), sign * -5);
        near(YawLimits_Speed(lo, 0, 0, -5, (float)sign, -1), sign * 5);
        near(YawLimits_Speed(0, 60, 0, 0, (float)sign, 0), 0);
        /* No discontinuity at the former 8-degree plateau transition. */
        near(YawLimits_Speed(hi - 8, sign * 100, 0, 0, (float)sign, -1), sign * 24);
        near(YawLimits_Speed(hi - 9, sign * 100, 0, 0, (float)sign, -1), sign * 27);
    }
    near(YawLimits_Effort(hi, 4), 0);
    near(YawLimits_Effort(hi, -4), -4);
    near(YawLimits_Effort(lo, -4), 0);
    near(YawLimits_Effort(lo, 4), 4);
    near(YawLimits_Effort(0, 4), 4);
    /* Repeated outward requests cannot accumulate past the endpoint. */
    joint = 0;
    for (int i = 0; i < 10000; ++i)
        joint = YawLimits_Error(hi, joint + 0.3f, 1);
    near(joint, 0);
    near(YawLimits_Error(hi, joint - 0.3f, 1), -0.3f);
    puts("PASS: asymmetric yaw targets, speed envelope, recovery and effort guards");
    return 0;
}
