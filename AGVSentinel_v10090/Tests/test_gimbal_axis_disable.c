#include <assert.h>
#include <stdio.h>
#include "module_gimbal_axis.h"

typedef struct {
    float pending;
    float transmitted;
    unsigned disable_calls;
    unsigned flush_calls;
} Driver;

static int8_t disable(void *context)
{
    Driver *driver = context;
    driver->pending = 0;
    driver->disable_calls++;
    return ACTUATOR_OK;
}

static int8_t effort(void *context, float value)
{
    ((Driver *)context)->pending = value;
    return ACTUATOR_OK;
}

static int8_t flush(void *context)
{
    Driver *driver = context;
    driver->transmitted = driver->pending;
    driver->flush_calls++;
    return ACTUATOR_OK;
}

int main(void)
{
    Driver big = {0}, small = {0};
    const Actuator_MotorOpsTypeDef ops = {
        .disable = disable, .set_effort = effort, .flush = flush
    };
    Actuator_MotorTypeDef big_motor, small_motor;
    Actuator_GroupTypeDef big_group = {"big", flush, &big};
    Actuator_GroupTypeDef small_group = {"small", flush, &small};
    GimbalAxis_TypeDef big_axis, small_axis;
    Actuator_MotorBind(&big_motor, "big", &ops, &big);
    Actuator_MotorBind(&small_motor, "small", &ops, &small);
    GimbalAxis_Init(&big_axis, &big_motor, &big_group, 0);
    GimbalAxis_Init(&small_axis, &small_motor, &small_group, 0);
    GimbalAxis_SetEffort(&big_axis, 30);
    GimbalAxis_SetEnabled(&big_axis, 0);
    assert(big.pending == 0 && big.disable_calls == 1);
    for (int i = 0; i < 1000; ++i) {
        /* Disabled big yaw rejects a stale or accidentally recomputed effort. */
        GimbalAxis_SetEffort(&big_axis, i % 2 ? 30 : -30);
        GimbalAxis_SetEnabled(&big_axis, 0);
        GimbalAxis_Flush(&big_axis);
        GimbalAxis_SetEffort(&small_axis, i % 2 ? 6 : -6);
        GimbalAxis_Flush(&small_axis);
        assert(big.pending == 0 && big.transmitted == 0);
        assert(small.transmitted == (i % 2 ? 6 : -6));
    }
    assert(big.flush_calls == 1000 && small.flush_calls == 1000);
    assert(big.disable_calls == 1 && small.disable_calls == 0);
    puts("PASS: disabled big axis transmits zero while small axis remains independent");
    return 0;
}
