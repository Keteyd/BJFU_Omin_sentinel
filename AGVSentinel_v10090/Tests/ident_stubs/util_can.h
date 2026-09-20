#ifndef IDENT_STUB_CAN_H
#define IDENT_STUB_CAN_H
#include "module_can_trace.h"
void Can_TraceBegin(void);
void Can_TraceEnd(void);
uint32_t Can_TraceSnapshot(uint32_t *now_us, CanTrace_AxisRecord out[2]);
#endif
