#ifndef MODULE_YAW_IDENT_WIRE_H
#define MODULE_YAW_IDENT_WIRE_H
#include <stdint.h>
#include <string.h>
#include "module_remote_intent.h"
#include "module_can_trace.h"

#define YAW_IDENT_MAGIC 0x31444959UL
#define YAW_IDENT_BUILD 0x59490910UL
#define YAW_IDENT_WIRE_SAMPLES 1001U
#define YAW_IDENT_STREAM_MAX_RECEIPT 9001U
#define YAW_IDENT_META_PARTS 24U
#define YAW_IDENT_RECORD_PARTS 6U
#define YAW_IDENT_CMD_REQUEST 0x36U
#define YAW_IDENT_CMD_STATUS 0x37U
#define YAW_IDENT_CMD_ACK 0x38U
#define YAW_IDENT_CMD_META 0x39U
#define YAW_IDENT_CMD_SAMPLE 0x3AU
#define YAW_IDENT_CMD_INFO 0x3BU
#define YAW_IDENT_CMD_REMOTE 0x3CU
#define YAW_IDENT_REMOTE_PARTS 3U
#define YAW_IDENT_REMOTE_VERSION 2U
#define YAW_IDENT_CMD_STREAM 0x3DU
#define YAW_IDENT_CMD_PROFILE 0x3EU
#define YAW_IDENT_PROFILE_PARTS 8U
#define YAW_IDENT_STREAM_VERSION 5U

enum { YAW_IDENT_PROBE = 0, YAW_IDENT_ARM = 1, YAW_IDENT_KEEPALIVE = 2,
       YAW_IDENT_CANCEL = 3, YAW_IDENT_DOWNLOAD = 4, YAW_IDENT_RELEASE = 5,
       YAW_IDENT_ARM_SLOW = 6, YAW_IDENT_ARM_SLOW_REVERSE = 7,
       YAW_IDENT_BENCH = 8, YAW_IDENT_STREAM_ACK = 9,
       YAW_IDENT_ARM_DUAL_A = 10, YAW_IDENT_ARM_DUAL_B = 11,
       YAW_IDENT_ARM_CD_C = 12, YAW_IDENT_ARM_CD_D = 13,
       YAW_IDENT_ARM_CD_E = 14,
       YAW_IDENT_ARM_SPEED_S1 = 15, YAW_IDENT_ARM_SPEED_S2 = 16,
       YAW_IDENT_ARM_SPEED_S3 = 17 };
enum { YAW_IDENT_ACCEPTED = 1, YAW_IDENT_REJECTED = 2, YAW_IDENT_BUSY = 3,
       YAW_IDENT_UNSAFE = 4, YAW_IDENT_BAD_PACKET = 5 };

typedef struct {
    uint32_t id;
    uint16_t amplitude_cdeg;
    uint8_t op, axis;
    uint32_t magic;
} YawIdent_Packet;

typedef struct {
    uint32_t tick_ms;
    float yaw_deg, gyro[3], roll_deg, pitch_deg;
    uint16_t big_raw, small_raw;
    int16_t big_rpm, small_rpm, big_command, small_command, offset_cdeg;
    uint16_t flags;
    uint8_t imu_age_ms, big_age_ms, small_age_ms, phase;
} YawIdent_Record;

typedef struct {
    YawIdent_Record sample;
    uint32_t trace_us, interval_us;
    CanTrace_AxisRecord axes[2];
} YawIdent_TraceRecord;
typedef char YawIdentTraceSize[(sizeof(YawIdent_TraceRecord) == CAN_TRACE_BYTES) ? 1 : -1];

typedef struct {
    uint32_t id, start_ms, build;
    uint16_t count, period_ms;
    uint8_t phase, reason, axis, version;
    uint32_t setup; /* 1: fixed chassis, usual Pitch pose, camera absent. */
    float values[42];
} YawIdent_Metadata;

typedef struct {
    uint32_t tick_ms;
    int16_t channels[5], mouse_x, mouse_y;
    uint8_t mouse_left, mouse_right;
    uint16_t age_ms, failed_mask;
} YawIdent_RemoteRecord;

/* Immutable descriptor for the active stream. Times include baseline and settle. */
typedef struct {
    uint32_t id, build, baud;
    uint16_t samples, period_ms, capacity, duration_ms;
    uint16_t knots_ms[8];
    float big_peak, small_peak, big_travel, small_travel, heading_travel, center_deg;
    uint8_t profile, reverse, version, reserved;
} YawIdent_ProfileRecord;

/* The same mask governs arming and diagnostics; no second neutral threshold. */
static inline uint16_t YawIdent_NeutralMask(const int16_t channels[5],
    int16_t mouse_x, int16_t mouse_y, uint8_t left, uint8_t right)
{
    unsigned i;
    uint16_t mask = 0U;
    for (i = 0U; i < 4U; ++i)
        if (channels[i] < -10 || channels[i] > 10) mask |= (uint16_t)(1U << i);
    /* The auxiliary wheel is not a motion input; reject its fire request. */
    if (channels[4] > REMOTE_WHEEL_FIRE_THRESHOLD) mask |= 1U << 4;
    if (mouse_x) mask |= 1U << 5;
    if (mouse_y) mask |= 1U << 6;
    if (left) mask |= 1U << 7;
    if (right) mask |= 1U << 8;
    return mask;
}

typedef char YawIdent_PacketSize[(sizeof(YawIdent_Packet) == 12U) ? 1 : -1];
typedef char YawIdent_RecordSize[(sizeof(YawIdent_Record) == 48U) ? 1 : -1];
typedef char YawIdent_MetadataSize[(sizeof(YawIdent_Metadata) == 192U) ? 1 : -1];
typedef char YawIdent_RemoteSize[(sizeof(YawIdent_RemoteRecord) == 24U) ? 1 : -1];
typedef char YawIdent_ProfileSize[(sizeof(YawIdent_ProfileRecord) == 64U) ? 1 : -1];

static inline uint8_t YawIdent_DecodeRequest(const uint8_t payload[12], YawIdent_Packet *p)
{
    memcpy(p, payload, 12U);
    if (p->magic != YAW_IDENT_MAGIC || p->op > YAW_IDENT_ARM_SPEED_S3) return 0U;
    if (p->op == YAW_IDENT_ARM)
        return (uint8_t)(p->id != 0U && p->axis >= 1U && p->axis <= 2U &&
                         p->amplitude_cdeg > 0U && p->amplitude_cdeg <= 500U);
    if (p->op == YAW_IDENT_ARM_SLOW || p->op == YAW_IDENT_ARM_SLOW_REVERSE)
        return (uint8_t)(p->id != 0U && (p->axis == 1U || p->axis == 2U) &&
            p->amplitude_cdeg > 0U && p->amplitude_cdeg <= (p->axis == 1U ? 1500U : 1000U));
    if (p->op == YAW_IDENT_STREAM_ACK)
        return (uint8_t)(p->id != 0U && p->axis == 0U &&
            p->amplitude_cdeg > 0U && p->amplitude_cdeg <= YAW_IDENT_STREAM_MAX_RECEIPT);
    if (p->op == YAW_IDENT_ARM_DUAL_A || p->op == YAW_IDENT_ARM_DUAL_B)
        return (uint8_t)(p->id != 0U && p->axis == 3U && p->amplitude_cdeg == 0U);
    if (p->op == YAW_IDENT_ARM_CD_C || p->op == YAW_IDENT_ARM_CD_D ||
        p->op == YAW_IDENT_ARM_CD_E)
        return (uint8_t)(p->id != 0U && p->axis == 4U && p->amplitude_cdeg == 0U);
    if (p->op == YAW_IDENT_ARM_SPEED_S1 || p->op == YAW_IDENT_ARM_SPEED_S2 ||
        p->op == YAW_IDENT_ARM_SPEED_S3)
        return (uint8_t)(p->id != 0U && p->axis == 5U && p->amplitude_cdeg == 0U);
    return (uint8_t)(p->axis == 0U && p->amplitude_cdeg == 0U &&
                    (p->op == YAW_IDENT_PROBE || p->id != 0U));
}

static inline void YawIdent_Part(const void *data, uint16_t sequence, uint8_t part,
                                uint8_t payload[12])
{
    payload[0] = (uint8_t)sequence;
    payload[1] = (uint8_t)(sequence >> 8);
    payload[2] = part;
    payload[3] = 1U;
    memcpy(payload + 4U, (const uint8_t *)data + part * 8U, 8U);
}
#endif
