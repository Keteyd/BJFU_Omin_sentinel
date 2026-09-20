#include <assert.h>
#include <stdio.h>
#define YAW_COORD_IMMEDIATE_FOLLOW 0U
#include "module_yaw_coordinator.h"

static void near(float a, float b) { assert(fabsf(a - b) < .0002f); }

static void tick(YawCoordinator_State *s, float q, float rate, float *big, int track)
{
    assert(YawCoordinator_Step(s, q, rate, *big, 1, .001f, 1));
    assert(isfinite(s->velocity_dps) && isfinite(s->feedforward_dps));
    assert(fabsf(s->feedforward_dps) <= fabsf(s->velocity_dps) + .0001f);
    assert(s->reference_deg >= 0 && s->reference_deg < 360);
    if (track) *big = s->reference_deg;
}

int main(void)
{
    YawCoordinator_State s = {0};
    float big = 359.9f, hold, total;
    int i;
    near(YAW_COORD_LEFT_ENTER_DEG, 20.3560547f);
    near(YAW_COORD_RIGHT_ENTER_DEG, -39.6744141f);
    near(YAW_LIMIT_ZERO_DEG, 6816.0f * 360 / 8192);
    tick(&s, 0, 0, &big, 1);
    for (i = 0; i < 1000; ++i) tick(&s, 5*sinf(i*.01f), 40, &big, 1);
    assert(s.mode == YAW_COORD_HOLD);
    near(s.reference_deg, 359.9f);
    big = 4;
    tick(&s, 0, 0, &big, 0);
    near(s.reference_deg, 359.9f); /* Disturbance cannot recapture HOLD. */

    YawCoordinator_Reset(&s); big = 100;
    tick(&s, 0, 0, &big, 1);
    for (int cycle = 0; cycle < 5; ++cycle) {
        for (i = 0; i < 80; ++i) tick(&s, 23, 0, &big, 1);
        assert(s.mode == YAW_COORD_HOLD);
        for (i = 0; i < 100; ++i) tick(&s, 0, 0, &big, 1);
    }
    for (i = 0; i < 150; ++i) tick(&s, 23, 0, &big, 1);
    assert(s.mode == YAW_COORD_LEFT && s.velocity_dps > 0);
    for (i = 0; i < 100; ++i) tick(&s, 18, 0, &big, 1);
    assert(s.mode == YAW_COORD_LEFT);
    for (i = 0; i < 700; ++i) tick(&s, 10, 0, &big, 1);
    assert(s.mode == YAW_COORD_HOLD);
    hold = s.reference_deg;
    for (i = 0; i < 500; ++i) tick(&s, 0, 0, &big, 1);
    near(s.reference_deg, hold);
    for (i = 0; i < 500; ++i) tick(&s, -30, 0, &big, 1);
    assert(s.mode == YAW_COORD_HOLD);
    for (i = 0; i < 150; ++i) tick(&s, -42, 0, &big, 1);
    assert(s.mode == YAW_COORD_RIGHT && s.velocity_dps < 0);

    for (int level = 7; level <= 10; ++level) {
        YawCoordinator_State left = {0}, right = {0};
        float lb = 100, rb = 100, ratio = level*.1f;
        tick(&left, 0, 0, &lb, 1); tick(&right, 0, 0, &rb, 1);
        for (i = 0; i < 1000; ++i) {
            tick(&left, YAW_LIMIT_MAX_DEG*ratio, 0, &lb, 1);
            tick(&right, YAW_LIMIT_MIN_DEG*ratio, 0, &rb, 1);
        }
        near(left.velocity_dps, -right.velocity_dps);
        if (level == 10) assert(left.velocity_dps > 60);
    }
    YawCoordinator_Reset(&s); tick(&s, 0, 0, &big, 1);
    tick(&s, YAW_LIMIT_MAX_DEG-1, 0, &big, 1);
    assert(s.mode == YAW_COORD_LEFT); /* Urgent relief bypasses dwell. */
    for (i = 0; i < 1000; ++i) tick(&s, YAW_LIMIT_MIN_DEG+1, 0, &big, 1);
    assert(s.mode == YAW_COORD_RIGHT && s.velocity_dps < -36);
    YawCoordinator_Reset(&s); tick(&s, 0, 0, &big, 1);
    for (i = 0; i < 60; ++i) tick(&s, YAW_LIMIT_MAX_DEG-4, 80, &big, 1);
    assert(s.mode == YAW_COORD_LEFT); /* Prediction before dwell. */

    YawCoordinator_Reset(&s); big = 359.9f; total = 0;
    tick(&s, 0, 0, &big, 1);
    for (i = 0; i < 100000; ++i) {
        float before = s.reference_deg;
        tick(&s, YAW_LIMIT_MAX_DEG-1, 0, &big, 1);
        total += remainderf(s.reference_deg-before, 360);
    }
    assert(total > 720);
    for (i = 0; i < 10000; ++i) tick(&s, YAW_LIMIT_MAX_DEG-1, 0, &big, 0);
    assert(fabsf(remainderf(s.reference_deg-big, 360)) <= YAW_COORD_POSITION_LEAD_DEG+.0001f);
    near(s.feedforward_dps, 0);
    for (int side = -1; side <= 1; side += 2) {
        float q = side > 0 ? 25 : -45, rate = 0;
        YawCoordinator_Reset(&s); big = 100;
        for (i = 0; i < 5000; ++i) {
            float before = big;
            tick(&s, q, rate, &big, 1);
            rate = -remainderf(big-before, 360)/.001f;
            q += rate*.001f;
        }
        assert(s.mode == YAW_COORD_HOLD && s.velocity_dps == 0);
        if (side > 0) assert(q > 0 && q < YAW_COORD_LEFT_EXIT_DEG+1);
        else assert(q < 0 && q > YAW_COORD_RIGHT_EXIT_DEG-1);
    }
    assert(!YawCoordinator_Step(&s, 0, 0, 123, 1, .001f, 0));
    assert(!s.ready && s.mode == YAW_COORD_DISABLED);
    assert(YawCoordinator_Step(&s, 0, 0, 123, -1, .001f, 1));
    assert(YawCoordinator_Step(&s, YAW_LIMIT_MAX_DEG, 0, 123, -1, .001f, 1));
    assert(s.velocity_dps < 0);
    assert(YawCoordinator_Step(&s, 0, 0, 123, 1, .001f, 1));
    near(s.velocity_dps, 0);
    assert(!YawCoordinator_Step(&s, NAN, 0, 123, 1, .001f, 1));
    assert(!YawCoordinator_Step(&s, 0, INFINITY, 123, 1, .001f, 1));
    assert(!YawCoordinator_Step(&s, 0, 0, NAN, 1, .001f, 1));
    assert(!YawCoordinator_Step(&s, 0, 0, 123, 0, .001f, 1));
    assert(!YawCoordinator_Step(&s, 0, 0, 123, 1, .051f, 1));
    puts("PASS: restored HOLD/relief, hysteresis, prediction, symmetry, wrap and SAFE");
    return 0;
}
