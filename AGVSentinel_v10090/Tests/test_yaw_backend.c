#include <assert.h>

#include "app_yaw_backend.h"

int main(void)
{
    assert(ControlYawBackend_Select(0U, 1U, 0U, 0U, 0U) ==
           CONTROL_YAW_BACKEND_DISABLED);
    assert(ControlYawBackend_Select(1U, 1U, 0U, 0U, 0U) ==
           CONTROL_YAW_BACKEND_MPC);
    assert(ControlYawBackend_Select(1U, 1U, 1U, 0U, 0U) ==
           CONTROL_YAW_BACKEND_PID);
    assert(ControlYawBackend_Select(1U, 0U, 0U, 0U, 0U) ==
           CONTROL_YAW_BACKEND_PID);
    assert(ControlYawBackend_Select(1U, 1U, 0U, 1U, 0U) ==
           CONTROL_YAW_BACKEND_PID);
    assert(ControlYawBackend_Select(1U, 1U, 0U, 0U, 1U) ==
           CONTROL_YAW_BACKEND_PID);
    return 0;
}
