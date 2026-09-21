#include "app_gimbal.h"

#include "app_autoaim.h"
#include "module_gimbal.h"
#include "module_shoot.h"

void Gimbal_Task(void const *argument)
{
    (void)argument;

    MyUART_Init();
    for (;;) {
        Tidy_send_vision(&visionDataSend);
        SendVisionData(&visionDataSend);
        Shooter_UpdataControlData();
        GimbalPitch_Control();
        GimbalPitch_Output();
        GimbalYaw_Control();
        Shooter_FeederControl();
        GimbalYaw_Output();
        osDelay(1U);
    }
}
