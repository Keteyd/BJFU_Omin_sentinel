#ifndef APP_COMMUNICATE_H
#define APP_COMMUNICATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_board_config.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdint.h>

typedef enum {
    COMM_CHASSIS_MODE_STOP = 0,
    COMM_CHASSIS_MODE_SEPARATE = 1,
    COMM_CHASSIS_MODE_FOLLOW = 2,
    COMM_CHASSIS_MODE_BODY = 3,
    COMM_CHASSIS_MODE_GIMBAL = 4
} Comm_ChassisModeEnum;

typedef struct {
    int16_t vx_ref;
    int16_t vy_ref;
    int16_t wz_ref;
    uint8_t mode;
    uint8_t sequence;
    uint32_t last_update_time;
} Comm_ChassisCommandTypeDef;

void Comm_Task(void const *argument);
void Comm_BoardLinkInit(void);
void Comm_BoardLinkTxStep(void);
void Comm_BoardLinkRxCanFrame(uint32_t stdid, const uint8_t *rxdata, uint32_t len);

void Comm_BoardLinkSetChassisCommand(const Comm_ChassisCommandTypeDef *command);
const Comm_ChassisCommandTypeDef *Comm_BoardLinkGetChassisCommand(void);
uint8_t Comm_BoardLinkRemoteOnline(void);
uint8_t Comm_BoardLinkChassisOnline(void);
uint8_t Comm_BoardLinkChassisCommandOnline(void);
uint8_t Comm_BoardLinkGimbalFeedbackOnline(void);
uint8_t Comm_BoardLinkRefereeOnline(void);
float Comm_BoardLinkGimbalRelYawDeg(void);
float Comm_BoardLinkGimbalPitchRad(void);

#ifdef __cplusplus
}
#endif

#endif
