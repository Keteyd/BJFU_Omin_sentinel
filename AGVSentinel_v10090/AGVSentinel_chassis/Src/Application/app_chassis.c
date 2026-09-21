#include "app_chassis.h"
#include "module_chassis.h"
#include "module_power.h"

/**
  * @brief          Chassis task
  * @param          NULL
  * @retval         NULL
  */
void Chassis_Task(void const * argument) {

    for(;;) {
		SuperCap_CheckPower(&SuperCap);
			Chassis_Control();
      Chassis_Output();//CAN下发四个电机输出
      osDelay(2);
    }
}
