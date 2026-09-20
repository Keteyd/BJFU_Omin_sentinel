#include <assert.h>
#include <math.h>
#include <stdint.h>

#include "app_chassis_frame.h"

static void expect_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
    float body_forward;
    float body_left;

    assert(!ChassisFrame_RemoteTranslationActive(10, -10, 10));
    assert(ChassisFrame_RemoteTranslationActive(11, 0, 10));
    assert(ChassisFrame_RemoteTranslationActive(0, -11, 10));

    assert(ChassisFrame_SmallYawToBody(100.0f, 20.0f, 0.0f, 0.0f,
                                      1.0f, &body_forward, &body_left));
    expect_close(body_forward, 100.0f);
    expect_close(body_left, 20.0f);

    assert(ChassisFrame_SmallYawToBody(100.0f, 0.0f, 30.0f, 60.0f,
                                      1.0f, &body_forward, &body_left));
    expect_close(body_forward, 0.0f);
    expect_close(body_left, 100.0f);

    assert(ChassisFrame_SmallYawToBody(100.0f, 0.0f, 30.0f, 15.0f,
                                      -1.0f, &body_forward, &body_left));
    expect_close(body_forward, 70.710678f);
    expect_close(body_left, -70.710678f);

    body_forward = 1.0f;
    body_left = 1.0f;
    assert(!ChassisFrame_SmallYawToBody(NAN, 0.0f, 0.0f, 0.0f,
                                       1.0f, &body_forward, &body_left));
    expect_close(body_forward, 0.0f);
    expect_close(body_left, 0.0f);
    assert(!ChassisFrame_SmallYawToBody(1.0f, 0.0f, 0.0f, 0.0f,
                                       0.0f, &body_forward, &body_left));
    return 0;
}
