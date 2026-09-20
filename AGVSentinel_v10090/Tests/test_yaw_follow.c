#include <assert.h>
#include <stdio.h>
#include "module_yaw_coordinator.h"

static void near(float a, float b) { assert(fabsf(a - b) < .0002f); }

static void step(YawCoordinator_State *s, float small, float big, float sign)
{
    assert(YawCoordinator_Step(s, small, 100, big, sign, .001f, 1));
    assert(s->ready && s->reference_deg >= 0 && s->reference_deg < 360);
    near(remainderf(s->reference_deg - big, 360), sign * small);
    near(s->feedforward_dps, 0);
    near(s->velocity_dps, 0);
    near(s->confirm_s, 0);
}

int main(void)
{
    YawCoordinator_State s = {0};
    assert(YAW_COORD_IMMEDIATE_FOLLOW == 1U);
    near(YAW_LIMIT_ZERO_DEG, 6816.0f * 360 / 8192);
    /* First tick follows immediately; no dwell, dead zone or 4-degree lead cap. */
    step(&s, 10, 355, 1);
    near(s.reference_deg, 5);
    assert(s.mode == YAW_COORD_LEFT);
    step(&s, -10, 5, 1);
    near(s.reference_deg, 355);
    assert(s.mode == YAW_COORD_RIGHT);
    step(&s, .01f, 100, 1);
    step(&s, -.01f, 100, 1);
    step(&s, 0, 123, 1);
    assert(s.mode == YAW_COORD_HOLD);
    step(&s, 0, 150, 1);
    near(s.reference_deg, 150); /* Centered follow is not a captured encoder HOLD. */
    for (int sign = -1; sign <= 1; sign += 2) {
        step(&s, YAW_LIMIT_HARD_MIN_DEG, 2, sign);
        step(&s, YAW_LIMIT_HARD_MAX_DEG, 358, sign);
        /* Ideal inertial small-yaw compensation returns its joint to center. */
        float small = -30, big = 100;
        step(&s, small, big, sign);
        small -= sign * remainderf(s.reference_deg - big, 360);
        near(small, 0);
    }
    float big = 355, total = 0;
    for (int i = 0; i < 100; ++i) {
        step(&s, 10, big, 1);
        total += remainderf(s.reference_deg - big, 360);
        big = s.reference_deg;
    }
    near(total, 1000);
    assert(!YawCoordinator_Step(&s, 1, 0, 100, 1, .001f, 0));
    assert(!s.ready && s.mode == YAW_COORD_DISABLED);
    near(s.reference_deg, 0);
    step(&s, -15, 220, -1);
    near(s.reference_deg, 235);
    assert(!YawCoordinator_Step(&s, NAN, 0, 100, 1, .001f, 1));
    assert(!YawCoordinator_Step(&s, 0, INFINITY, 100, 1, .001f, 1));
    assert(!YawCoordinator_Step(&s, 0, 0, NAN, 1, .001f, 1));
    assert(!YawCoordinator_Step(&s, 0, 0, -1, 1, .001f, 1));
    assert(!YawCoordinator_Step(&s, 0, 0, 360, 1, .001f, 1));
    assert(!YawCoordinator_Step(&s, 0, 0, 100, 0, .001f, 1));
    assert(!YawCoordinator_Step(&s, 0, 0, 100, 1, 0, 1));
    assert(!YawCoordinator_Step(&s, 0, 0, 100, 1, .051f, 1));
    assert(!YawCoordinator_Step(&s, YAW_LIMIT_HARD_MAX_DEG + 2, 0,
        100, 1, .001f, 1));
    assert(!YawCoordinator_Step(&s, YAW_LIMIT_HARD_MIN_DEG - 2, 0,
        100, 1, .001f, 1));
    assert(!s.ready && s.mode == YAW_COORD_DISABLED);
    puts("PASS: immediate centering, both signs, no dwell/lead cap, wrap and SAFE");
    return 0;
}
