#ifndef APP_YAW_BACKEND_H
#define APP_YAW_BACKEND_H

#include <stdint.h>

typedef enum {
    CONTROL_YAW_BACKEND_DISABLED = 0,
    CONTROL_YAW_BACKEND_PID = 1,
    CONTROL_YAW_BACKEND_MPC = 2
} Control_YawBackendEnum;

static inline Control_YawBackendEnum ControlYawBackend_Select(
    uint8_t gimbal_enabled, uint8_t mpc_requested,
    uint8_t operator_pid_selected, uint8_t tuning_locked,
    uint8_t small_yaw_angle_open_test)
{
    if (!gimbal_enabled) return CONTROL_YAW_BACKEND_DISABLED;
    if (operator_pid_selected || !mpc_requested || tuning_locked ||
        small_yaw_angle_open_test)
        return CONTROL_YAW_BACKEND_PID;
    return CONTROL_YAW_BACKEND_MPC;
}

#endif
