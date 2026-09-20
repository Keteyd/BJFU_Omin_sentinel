#include <assert.h>
#include <stdio.h>
#include "module_yaw_ident_wire.h"
int main(int argc, char **argv) {
    CanTrace_State state; CanTrace_AxisRecord out[2]; unsigned i;
    YawIdent_TraceRecord record = {0}; uint8_t frame[CAN_TRACE_FRAME_BYTES];
    uint8_t feedback[8] = {0x1f, 0xff, 0xff, 0xfe, 0xfe, 0xd4, 0, 0};
    CanTrace_Begin(&state);
    assert(CanTrace_Attempt(&state, 0, 1000, 100U) == 1U);
    CanTrace_Queue(&state, 0, 1U, 0, 1, 12, 0);
    CanTrace_Complete(&state, 0, 0, 250);
    CanTrace_Attempt(&state, 0, -500, 600U);
    CanTrace_Queue(&state, 0, 2U, 1, 1, 8, 0);
    CanTrace_Feedback(&state, 0, feedback, 900U);
    assert(CanTrace_Snapshot(&state, 1000U, out) == 1000U);
    assert(out[0].integral_raw_us == 300000 && out[0].known_us == 900);
    assert(out[0].minimum == -500 && out[0].maximum == 1000 && out[0].command == -500);
    assert(out[0].current == -300 && out[0].encoder == 8191 && out[0].rpm == -2);
    assert(out[0].complete_sequence == 1 && out[0].complete_us == 250 && out[0].wait_us == 20);
    assert(out[0].attempts == 2 && out[0].queued == 2 && out[0].completed == 1 && out[0].flags == 259);
    record.sample.tick_ms = 100; record.sample.flags = 29; record.sample.phase = 2;
    record.trace_us = record.interval_us = 1000;
    memcpy(record.axes, out, sizeof(out));
    CanTrace_Frame(frame, 1, 1, &record);
    if (argc > 1) {
        FILE *f = fopen(argv[1], "wb"); assert(f);
        assert(fwrite(frame, 1, sizeof(frame), f) == sizeof(frame)); fclose(f);
    }
    CanTrace_Complete(&state, 1, 1, 1100);
    assert(CanTrace_Snapshot(&state, 2000, out) == 1000);
    assert(out[0].integral_raw_us == -500000 && out[0].errors == 1 && out[0].attempts == 0);
    CanTrace_Attempt(&state, 0, 0, 2000);
    CanTrace_Queue(&state, 0, 3, 0, 0, 70000, 1);
    CanTrace_Snapshot(&state, 2000, out);
    assert(out[0].failed == 1 && (out[0].flags & (CT_WAIT_OVERFLOW | CT_WAIT_EXHAUSTED)) == 48);
    CanTrace_Begin(&state);
    for (i = 0; i < 260; ++i) CanTrace_Attempt(&state, 0, 30000, i);
    CanTrace_Snapshot(&state, 200000, out);
    assert(out[0].attempts == 255 && (out[0].flags & CT_COUNT_OVERFLOW));
    assert(out[0].integral_raw_us == INT32_MAX && (out[0].flags & CT_INTEGRAL_OVERFLOW));
    CanTrace_Begin(&state); state.boundary_us = 0xfffffff0U;
    state.axes[0].updated_us = state.axes[1].updated_us = state.boundary_us;
    CanTrace_Attempt(&state, 0, 2, 0xfffffff0U);
    assert(CanTrace_Snapshot(&state, 16U, out) == 32U && out[0].integral_raw_us == 64);
    state.active = 0; assert(!CanTrace_Attempt(&state, 0, 4, 100));
    puts("CAN trace core: integral, interval, overflow, wrap and wire layout passed.");
    return 0;
}
