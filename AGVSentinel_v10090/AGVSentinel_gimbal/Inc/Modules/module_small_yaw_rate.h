#ifndef MODULE_SMALL_YAW_RATE_H
#define MODULE_SMALL_YAW_RATE_H

#include <math.h>
#include <stdint.h>

/* IMU angle holding restored; set to 1 only for the angle-open diagnostic build. */
#define SMALL_YAW_ANGLE_OPEN_TEST 0U
#define SMALL_YAW_RATE_TIMEOUT_MS 20U

typedef struct {
    float rpm;
    uint32_t updated_ms;
    uint8_t valid;
} SmallYawRate_Command;

static inline void SmallYawRate_Reset(SmallYawRate_Command *command)
{
    command->rpm = 0.0f;
    command->updated_ms = 0U;
    command->valid = 0U;
}

static inline void SmallYawRate_SetDps(SmallYawRate_Command *command,
    float dps, uint32_t now)
{
    if (!isfinite(dps)) {
        SmallYawRate_Reset(command);
        return;
    }
    command->rpm = dps / 6.0f;
    command->updated_ms = now;
    command->valid = 1U;
}

static inline float SmallYawRate_GetRpm(SmallYawRate_Command *command,
    uint32_t now)
{
    if (!command->valid ||
        (uint32_t)(now - command->updated_ms) > SMALL_YAW_RATE_TIMEOUT_MS) {
        SmallYawRate_Reset(command);
        return 0.0f;
    }
    return command->rpm;
}

#endif
