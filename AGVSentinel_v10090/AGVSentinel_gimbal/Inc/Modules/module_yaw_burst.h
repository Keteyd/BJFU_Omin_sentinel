#ifndef MODULE_YAW_BURST_H
#define MODULE_YAW_BURST_H
#include "module_yaw_capture.h"

#define YAW_BURST_COUNT 512U
#define YAW_BURST_PERIOD_MS 2U
#define YAW_BURST_PARTS 12U
typedef struct {
    YawCapture_Record observation;
    float control_dt_s;
    float gimbal_dt_s;
} YawBurst_Record;
typedef char YawBurst_SizeCheck[sizeof(YawBurst_Record) == 96U ? 1 : -1];
typedef struct {
    YawBurst_Record records[YAW_BURST_COUNT];
    uint32_t last_ms;
    uint16_t count, sent, skipped;
    uint8_t part, started;
} YawBurst_Buffer;

static inline void YawBurst_Reset(YawBurst_Buffer *b)
{
    b->count = b->sent = b->skipped = 0U;
    b->part = b->started = 0U;
}
static inline uint8_t YawBurst_Due(YawBurst_Buffer *b, uint32_t now)
{
    uint32_t elapsed;
    if (b->count == YAW_BURST_COUNT) return 0U;
    if (!b->started) {
        b->started = 1U;
        b->last_ms = now;
        return 1U;
    }
    elapsed = now - b->last_ms;
    if (elapsed < YAW_BURST_PERIOD_MS) return 0U;
    b->skipped = (uint16_t)(b->skipped + elapsed / YAW_BURST_PERIOD_MS - 1U);
    b->last_ms = now;
    return 1U;
}
static inline uint8_t YawBurst_Payload(const YawBurst_Buffer *b, uint8_t p[12])
{
    if (b->count != YAW_BURST_COUNT || b->sent >= b->count) return 0U;
    p[0] = (uint8_t)(b->sent + 1U);
    p[1] = (uint8_t)((b->sent + 1U) >> 8);
    p[2] = b->part;
    p[3] = 2U;
    memcpy(p + 4, (const uint8_t *)&b->records[b->sent] + 8U * b->part, 8U);
    return 1U;
}
static inline void YawBurst_Accept(YawBurst_Buffer *b)
{
    if (b->count != YAW_BURST_COUNT || b->sent >= b->count) return;
    if (++b->part == YAW_BURST_PARTS) {
        b->part = 0U;
        ++b->sent;
    }
}
#endif
