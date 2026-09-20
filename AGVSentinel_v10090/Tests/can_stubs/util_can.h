#ifndef CAN_FIXTURE_H
#define CAN_FIXTURE_H
#include <stdint.h>
#include "module_can_trace.h"
typedef struct { uint32_t TSR, ESR; } Registers;
typedef struct { Registers *Instance; } CAN_HandleTypeDef;
typedef struct { uint32_t StdId, ExtId, RTR, IDE, DLC, TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef CAN_TxHeaderTypeDef CAN_RxHeaderTypeDef;
typedef struct { uint32_t FilterBank, FilterMode, FilterScale, FilterIdHigh, FilterIdLow,
    FilterMaskIdHigh, FilterMaskIdLow, FilterFIFOAssignment, FilterActivation, SlaveStartFilterBank; } CAN_FilterTypeDef;
typedef struct { uint32_t CTRL, CYCCNT; } Dwt;
typedef struct { uint32_t DEMCR; } Debug;
extern Dwt fake_dwt; extern Debug fake_debug;
#define DWT (&fake_dwt)
#define CoreDebug (&fake_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define SystemCoreClock 168000000U
#define CAN_RTR_DATA 0U
#define CAN_ID_STD 0U
#define DISABLE 0U
#define ENABLE 1U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 0U
#define CAN_RX_FIFO0 0U
#define CAN_RX_FIFO1 1U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define HAL_OK 0U
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TX_MAILBOX2 4U
#define CAN_FLAG_RQCP0 0x500U
extern CAN_HandleTypeDef hcan1, hcan2;
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
uint32_t HAL_CAN_ConfigFilter(CAN_HandleTypeDef*, CAN_FilterTypeDef*);
uint32_t HAL_CAN_Start(CAN_HandleTypeDef*);
uint32_t HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, uint32_t);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef*);
uint32_t HAL_CAN_AddTxMessage(CAN_HandleTypeDef*, CAN_TxHeaderTypeDef*, uint8_t*, uint32_t*);
uint32_t HAL_CAN_GetRxMessage(CAN_HandleTypeDef*, uint32_t, CAN_RxHeaderTypeDef*, uint8_t*);
void Can_RxMessageCallback(CAN_HandleTypeDef*, CAN_RxHeaderTypeDef*, uint8_t*);
void fake_clear(CAN_HandleTypeDef*, uint32_t);
#define __HAL_CAN_CLEAR_FLAG(bus, flag) fake_clear(bus, flag)
#endif
