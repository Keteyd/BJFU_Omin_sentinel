#ifndef MODULE_YAW_CAPTURE_H
#define MODULE_YAW_CAPTURE_H

#include <stdint.h>
#include <string.h>

#define YAW_CAPTURE_PERIOD_MS 25U
#define YAW_CAPTURE_PARTS 11U
#define YAW_CAPTURE_VERSION 1U
#define YAW_CAPTURE_SESSION_TAG 0xD1U

/* STM32 little-endian float32 wire record; observations are latest available,
 * not simultaneous sensor conversions. Ages and IMU counter expose skew. */
typedef struct {
    uint32_t tick_ms;
    uint32_t imu_sequence;
    uint16_t imu_age_ms, big_age_ms, small_age_ms;
    int16_t rc_yaw;
    uint16_t flags, skipped;
    float imu_dt_s;
    float yaw_deg, roll_deg, gyro_x_rad_s, gyro_y_rad_s, gyro_z_rad_s;
    float big_encoder_deg, small_encoder_deg, big_rpm, small_rpm;
    float big_command, small_command, yaw_target_deg;
    float big_current_raw, small_current_raw;
    float pitch_deg, small_speed_target_rpm;
} YawCapture_Record;
typedef char YawCapture_SizeCheck[sizeof(YawCapture_Record) == 88U ? 1 : -1];

static inline uint16_t YawCapture_Age(uint32_t now, uint32_t then)
{
    uint32_t age = now - then;
    return (uint16_t)(age > 65535U ? 65535U : age);
}

static inline void YawCapture_Part(const YawCapture_Record *record,
    uint16_t sequence, uint8_t part, uint8_t payload[12])
{
    payload[0] = (uint8_t)sequence;
    payload[1] = (uint8_t)(sequence >> 8);
    payload[2] = part;
    payload[3] = YAW_CAPTURE_VERSION;
    memcpy(payload + 4, (const uint8_t *)record + 8U * part, 8U);
}
#endif
