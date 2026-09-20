#include <assert.h>
#include <stdio.h>
#include "module_big_yaw_manual.h"

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < .0001f);
}

int main(void)
{
    BigYawManual_Command c = {0};
    BigYawManual_State s = {0};
    uint32_t generation;
    unsigned i;

    assert(!BigYawManual_Step(&s, &c, 0U, 1U, 354.0f, 0.0f, .002f));
    BigYawManual_Publish(&c, 1U, 100.0f, 100U);
    assert(BigYawManual_Step(&s, &c, 100U, 1U, 354.0f, 0.0f, .002f));
    near(s.reference_deg, -6.0f); /* Entry captures, even with a deflected stick. */
    generation = c.generation;
    BigYawManual_Publish(&c, 1U, 100.0f, 102U);
    assert(c.generation == generation);
    assert(BigYawManual_Step(&s, &c, 102U, 1U, 354.0f, 0.0f, .002f));
    near(s.reference_deg, -5.8f);
    BigYawManual_Publish(&c, 1U, -100.0f, 104U);
    assert(BigYawManual_Step(&s, &c, 104U, 1U, 354.0f, 0.0f, .004f));
    near(s.reference_deg, -6.2f);
    BigYawManual_Publish(&c, 1U, 0.0f, 106U);
    assert(BigYawManual_Step(&s, &c, 106U, 1U, 350.0f, -5.0f, .002f));
    near(s.reference_deg, -6.2f); /* Center holds target, not measured pose. */
    BigYawManual_Publish(&c, 1U, -2.0f, 106U);
    near(c.rate_dps, 0.0f);
    BigYawManual_Publish(&c, 1U, 2.0f, 106U);
    near(c.rate_dps, 0.0f);
    assert(!BigYawManual_Step(&s, &c, 157U, 1U, 350.0f, 0.0f, .002f));
    assert(!s.active && !s.fault);
    BigYawManual_Publish(&c, 1U, 0.0f, 158U);
    assert(BigYawManual_Step(&s, &c, 158U, 1U, 300.0f, 0.0f, .002f));
    near(s.reference_deg, -60.0f);

    /* A rapid off/on between control iterations must still recapture. */
    BigYawManual_Publish(&c, 0U, 0.0f, 159U);
    BigYawManual_Publish(&c, 1U, 0.0f, 160U);
    assert(BigYawManual_Step(&s, &c, 160U, 1U, 250.0f, 0.0f, .002f));
    near(s.reference_deg, -110.0f);
    assert(!BigYawManual_Step(&s, &c, 161U, 0U, 250.0f, 0.0f, .002f));
    assert(!s.active);

    BigYawManual_Publish(&c, 1U, 132.0f, 200U);
    assert(BigYawManual_Step(&s, &c, 200U, 1U, 179.9f, 0.0f, .002f));
    for (i = 1; i <= 5000U; ++i) {
        BigYawManual_Publish(&c, 1U, 132.0f, 200U + 2U * i);
        assert(BigYawManual_Step(&s, &c, 200U + 2U * i, 1U, 0.0f, 22.0f, .002f));
        assert(fabsf(s.reference_deg) <= 180.0f);
    }

    /* Finite speed alone no longer aborts manual control, in either direction. */
    assert(BigYawManual_Step(&s, &c, c.tick_ms, 1U, 0.0f, 30.1f, .002f));
    assert(BigYawManual_Step(&s, &c, c.tick_ms, 1U, 0.0f, -30.1f, .002f));
    assert(BigYawManual_Step(&s, &c, c.tick_ms, 1U, 0.0f, 300.0f, .002f));
    assert(BigYawManual_Step(&s, &c, c.tick_ms, 1U, 0.0f, -300.0f, .002f));
    assert(!s.fault && s.active);
    assert(!BigYawManual_Step(&s, &c, c.tick_ms, 0U, 0.0f, 300.0f, .002f));
    assert(!s.active); /* Permission loss still stops at high speed. */
    assert(!BigYawManual_Step(&s, &c, c.tick_ms, 1U, 0.0f, NAN, .002f));
    assert(s.fault && !s.active);
    assert(!BigYawManual_Step(&s, &c, c.tick_ms, 1U, 0.0f, 0.0f, .002f));
    BigYawManual_Publish(&c, 0U, 0.0f, c.tick_ms);
    BigYawManual_Publish(&c, 1U, 0.0f, c.tick_ms);
    assert(!BigYawManual_Step(&s, &c, c.tick_ms, 1U, 0.0f, 0.0f, .002f));
    s.fault = 0U; /* Production clears only on fresh right-switch UP. */
    assert(BigYawManual_Step(&s, &c, c.tick_ms, 1U, 20.0f, 0.0f, .002f));
    assert(!BigYawManual_Step(&s, &c, c.tick_ms, 1U, NAN, 0.0f, .002f));
    s.fault = 0U;
    assert(!BigYawManual_Step(&s, &c, c.tick_ms, 1U, 0.0f, INFINITY, .002f));
    s.fault = 0U;
    assert(!BigYawManual_Step(&s, &c, c.tick_ms, 1U, 0.0f, 0.0f, .011f));
    s.fault = 0U;
    assert(!BigYawManual_Step(&s, &c, c.tick_ms, 1U, 0.0f, 0.0f, NAN));
    s.fault = 0U;
    BigYawManual_Publish(&c, 1U, NAN, 1U);
    assert(!BigYawManual_Step(&s, &c, 1U, 1U, 0.0f, 0.0f, .002f));
    BigYawManual_Publish(&c, 1U, 0.0f, UINT32_MAX - 4U);
    assert(BigYawManual_Fresh(&c, 45U));
    assert(!BigYawManual_Fresh(&c, 46U));
    puts("big yaw manual: PASS");
    return 0;
}
