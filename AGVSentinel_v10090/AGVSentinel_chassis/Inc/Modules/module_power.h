#ifndef MODULE_POWER
#define MODULE_POWER

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "periph_motor.h"
typedef struct {
    float input_voltage;  // 输入电压 (V)
    float cap_voltage;    // 电容电压 (V)
    float input_current;  // 输入电流 (A)
    float target_power;   // 目标功率 (W)
} SuperCap_Data_t;


void Parse_SuperCap_Feedback(uint8_t *rx_data, SuperCap_Data_t *cap_data);
void SuperCap_SetPower(CAN_HandleTypeDef* phcan, CAN_TxHeaderTypeDef* pheader, float target_power_w);
void SuperCap_InitHeader(CAN_TxHeaderTypeDef *pheader);
void SuperCap_CheckPower(SuperCap_Data_t *pcap_data);

extern CAN_TxHeaderTypeDef CapHeader;
extern SuperCap_Data_t SuperCap ;

#endif
	
#ifdef __cplusplus
}
#endif