#ifndef APP_CHASSIS_FRAME_H
#define APP_CHASSIS_FRAME_H

#include <math.h>
#include <stdint.h>

#define CHASSIS_FRAME_PI (3.14159265358979323846f)

static inline uint8_t ChassisFrame_RemoteTranslationActive(
    int16_t forward_count, int16_t left_count, int16_t deadband_count)
{
    int32_t forward = forward_count;
    int32_t left = left_count;
    int32_t deadband = deadband_count;

    if (deadband < 0) deadband = -deadband;
    if (forward < 0) forward = -forward;
    if (left < 0) left = -left;
    return (uint8_t)(forward > deadband || left > deadband);
}

/* Rotate a translation command expressed in the small-yaw frame into the
 * chassis body frame. calibrated_big_deg is the existing big-yaw angle after
 * chassis installation offset compensation; small_joint_deg is q = small-big.
 */
static inline uint8_t ChassisFrame_SmallYawToBody(
    float forward, float left, float calibrated_big_deg,
    float small_joint_deg, float direction_sign,
    float *body_forward, float *body_left)
{
    float heading_rad;
    float heading_cos;
    float heading_sin;

    if (body_forward == 0 || body_left == 0) return 0U;
    *body_forward = 0.0f;
    *body_left = 0.0f;
    if (!isfinite(forward) || !isfinite(left) ||
        !isfinite(calibrated_big_deg) || !isfinite(small_joint_deg) ||
        (direction_sign != 1.0f && direction_sign != -1.0f)) return 0U;

    heading_rad = (calibrated_big_deg + small_joint_deg) *
                  direction_sign * CHASSIS_FRAME_PI / 180.0f;
    heading_cos = cosf(heading_rad);
    heading_sin = sinf(heading_rad);
    *body_forward = forward * heading_cos - left * heading_sin;
    *body_left = forward * heading_sin + left * heading_cos;
    return (uint8_t)(isfinite(*body_forward) && isfinite(*body_left));
}

#endif
