/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_watchDog.c
 *  Description  : Watch Dog Application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:45:15
 *  LastEditTime : 2023-04-07 15:27:51
 */


#include "app_watchDog.h"
#include "periph_bmi088.h"
#include "periph_remote.h"
#include "protocol_common.h"
#include "app_board_config.h"
#include "sys_robot_actuators.h"

uint16_t Dog_Bowl = 0;

void WatchDog_Task(void const * argument) {
    WatchDog_Feed();
    osDelay(50);

    forever {
        WatchDog_Feed();
      osDelay(50);
    }
}


void WatchDog_Feed() {
    uint16_t dog_bowl = 0U;

#if BOARD_HAS_INS
    if (BMI088_IsBMI088Offline())
        dog_bowl |= 1U << 0;
#endif
    if (RobotActuators_AnyOffline())
        dog_bowl |= 1U << 1;
    /*
     * 去掉“遥控掉线检测”：不再把 Remote_STATE_LOST 计入 Dog_Bowl。
     * 这样遥控帧间歇丢失时不会触发看门狗安全停机，也避免“无法重连”的现象。
     */
    if (Protocol_IsProtocolOffline())
        dog_bowl |= 1U << 3;

    Dog_Bowl = dog_bowl;
}

uint8_t WatchDoge_CheckDogBowl() {
    return Dog_Bowl == 0 ? 0 : 1;
}
