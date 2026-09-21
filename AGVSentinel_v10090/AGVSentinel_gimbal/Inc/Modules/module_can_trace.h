#ifndef MODULE_CAN_TRACE_H
#define MODULE_CAN_TRACE_H
/* Pure bounded recorder. Callers serialize task/IRQ access. Units: raw CAN
 * command/current, microseconds since capture start. No motor commands here. */
#include <stdint.h>
#include <string.h>
#include <limits.h>
#define CAN_TRACE_CAPACITY 320U
#define CAN_TRACE_BYTES 152U
#define CAN_TRACE_FRAME_BYTES 163U
#define CAN_TRACE_CMD 0x3FU
enum { CT_COMMAND = 1, CT_FEEDBACK = 2, CT_COUNT_OVERFLOW = 4,
       CT_INTEGRAL_OVERFLOW = 8, CT_WAIT_OVERFLOW = 16, CT_WAIT_EXHAUSTED = 32,
       CT_SLOT_COLLISION = 64, CT_BUS_ERROR = 128 };
typedef struct {
    uint32_t sequence, attempt_us;
    int32_t integral_raw_us;
    uint32_t feedback_us, complete_sequence, complete_us;
    uint16_t known_us;
    int16_t minimum, maximum, command, current;
    uint16_t encoder;
    int16_t rpm;
    uint16_t wait_us, flags;
    uint8_t attempts, queued, completed, failed, errors, aborted;
} CanTrace_AxisRecord;
typedef char CanTraceAxisSize[(sizeof(CanTrace_AxisRecord) == 48U) ? 1 : -1];
typedef struct { CanTrace_AxisRecord r; uint32_t updated_us; } CanTrace_Axis;
typedef struct { uint32_t sequence; uint8_t axis; } CanTrace_Pending;
typedef struct {
    CanTrace_Axis axes[2];
    CanTrace_Pending pending[6];
    uint32_t boundary_us;
    uint8_t active;
} CanTrace_State;
static inline void CanTrace_Count(uint8_t *count, uint16_t *flags) {
    if (*count == 255U) *flags |= CT_COUNT_OVERFLOW; else ++*count;
}
static inline void CanTrace_Begin(CanTrace_State *s) {
    memset(s, 0, sizeof(*s)); s->active = 1U;
}
static inline void CanTrace_Advance(CanTrace_Axis *a, uint32_t now) {
    uint32_t dt = now - a->updated_us;
    if (a->r.flags & CT_COMMAND) {
        int64_t area = (int64_t)a->r.integral_raw_us + (int64_t)a->r.command * dt;
        uint64_t known = (uint64_t)a->r.known_us + dt;
        if (area > INT32_MAX || area < INT32_MIN || known > 65535U) a->r.flags |= CT_INTEGRAL_OVERFLOW;
        a->r.integral_raw_us = area > INT32_MAX ? INT32_MAX : area < INT32_MIN ? INT32_MIN : (int32_t)area;
        a->r.known_us = known > 65535U ? 65535U : (uint16_t)known;
    }
    a->updated_us = now;
}
static inline uint32_t CanTrace_Attempt(CanTrace_State *s, unsigned axis, int16_t command, uint32_t now) {
    CanTrace_Axis *a;
    if (!s->active || axis >= 2U) return 0U;
    a = &s->axes[axis]; CanTrace_Advance(a, now);
    if (!(a->r.flags & CT_COMMAND)) a->r.minimum = a->r.maximum = command;
    if (command < a->r.minimum) a->r.minimum = command;
    if (command > a->r.maximum) a->r.maximum = command;
    a->r.command = command; a->r.flags |= CT_COMMAND;
    a->r.attempt_us = now; ++a->r.sequence;
    CanTrace_Count(&a->r.attempts, &a->r.flags);
    return a->r.sequence;
}
static inline void CanTrace_Queue(CanTrace_State *s, unsigned axis, uint32_t sequence,
    unsigned slot, uint8_t success, uint32_t wait_us, uint8_t exhausted) {
    CanTrace_AxisRecord *r;
    if (!s->active || axis >= 2U || !sequence) return;
    r = &s->axes[axis].r;
    if ((uint64_t)r->wait_us + wait_us > 65535U) { r->wait_us = 65535U; r->flags |= CT_WAIT_OVERFLOW; }
    else r->wait_us += (uint16_t)wait_us;
    if (exhausted) r->flags |= CT_WAIT_EXHAUSTED;
    if (!success || slot >= 6U) { CanTrace_Count(&r->failed, &r->flags); return; }
    CanTrace_Count(&r->queued, &r->flags);
    if (s->pending[slot].axis) r->flags |= CT_SLOT_COLLISION;
    s->pending[slot].axis = (uint8_t)(axis + 1U); s->pending[slot].sequence = sequence;
}
/* kind 0=TXOK, 1=arbitration/transmit error, 2=abort. Timestamp is observation
 * time of the hardware completion flags, not the CAN wire edge. */
static inline void CanTrace_Complete(CanTrace_State *s, unsigned slot, unsigned kind, uint32_t now) {
    CanTrace_Pending *p;
    CanTrace_AxisRecord *r;
    if (!s->active || slot >= 6U) return;
    p = &s->pending[slot]; if (!p->axis) return;
    r = &s->axes[p->axis - 1U].r;
    if (kind == 0U) {
        CanTrace_Count(&r->completed, &r->flags);
        r->complete_sequence = p->sequence; r->complete_us = now;
    } else if (kind == 1U) CanTrace_Count(&r->errors, &r->flags);
    else CanTrace_Count(&r->aborted, &r->flags);
    memset(p, 0, sizeof(*p));
}
static inline void CanTrace_Feedback(CanTrace_State *s, unsigned axis, const uint8_t data[8], uint32_t now) {
    CanTrace_AxisRecord *r;
    if (!s->active || axis >= 2U) return;
    r = &s->axes[axis].r;
    r->encoder = (uint16_t)((data[0] << 8) | data[1]);
    r->rpm = (int16_t)((data[2] << 8) | data[3]);
    r->current = (int16_t)((data[4] << 8) | data[5]);
    r->feedback_us = now; r->flags |= CT_FEEDBACK;
}
static inline uint32_t CanTrace_Snapshot(CanTrace_State *s, uint32_t now, CanTrace_AxisRecord out[2]) {
    unsigned axis, slot;
    uint32_t duration = now - s->boundary_us;
    for (axis = 0U; axis < 2U; ++axis) {
        CanTrace_Axis *a = &s->axes[axis];
        CanTrace_Advance(a, now); out[axis] = a->r;
        for (slot = 0U; slot < 6U; ++slot)
            if (s->pending[slot].axis == axis + 1U) out[axis].flags += 1U << 8;
        a->r.integral_raw_us = 0; a->r.known_us = 0;
        a->r.minimum = a->r.maximum = a->r.command;
        a->r.wait_us = 0; a->r.flags &= CT_COMMAND | CT_FEEDBACK;
        a->r.attempts = a->r.queued = a->r.completed = a->r.failed = a->r.errors = a->r.aborted = 0;
    }
    s->boundary_us = now; return duration;
}
static inline uint16_t CanTrace_Crc16(const uint8_t *data, unsigned size) {
    uint16_t crc = 0xffffU; unsigned i, bit;
    for (i = 0; i < size; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0; bit < 8U; ++bit) crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
    }
    return crc;
}
static inline void CanTrace_Frame(uint8_t out[CAN_TRACE_FRAME_BYTES], uint32_t id, uint16_t seq, const void *record) {
    uint16_t crc;
    out[0] = 0xffU; out[1] = CAN_TRACE_CMD;
    memcpy(out + 2U, &id, 4U); memcpy(out + 6U, &seq, 2U);
    memcpy(out + 8U, record, CAN_TRACE_BYTES);
    crc = CanTrace_Crc16(out, CAN_TRACE_BYTES + 8U);
    memcpy(out + CAN_TRACE_BYTES + 8U, &crc, 2U); out[CAN_TRACE_FRAME_BYTES - 1U] = 13U;
}
#endif
