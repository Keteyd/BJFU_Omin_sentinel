/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_can.c
 *  Description  : This file contains the functions of CAN
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-05-06 19:51:17
 */


#include "util_can.h"
#include "protocol_common.h"
#include "app_communicate.h"
#include "sys_robot_actuators.h"

CAN_RxHeaderTypeDef Can_RxHeader;
#define Const_Can_RX_BUFF_LEN  200
uint8_t Can_RxData[Const_Can_RX_BUFF_LEN];

static CanTrace_State s_trace;
static uint32_t s_trace_cycles, s_trace_us, s_trace_remainder, s_cycles_per_us;
/* Called with interrupts masked. CYCCNT is never reset: other users keep their
 * epoch. Frequent calls during the <=20s trace handle uint32 cycle wrap. */
static uint32_t Can_TraceNow(void) {
    uint32_t cycles = DWT->CYCCNT;
    uint64_t delta = (uint32_t)(cycles - s_trace_cycles) + (uint64_t)s_trace_remainder;
    s_trace_cycles = cycles;
    s_trace_us += (uint32_t)(delta / s_cycles_per_us);
    s_trace_remainder = (uint32_t)(delta % s_cycles_per_us);
    return s_trace_us;
}
static void Can_TracePollBus(CAN_HandleTypeDef *bus, unsigned base, uint32_t now) {
    uint32_t tsr = bus->Instance->TSR; unsigned i;
    if (bus->Instance->ESR & 7U) s_trace.axes[base ? 1U : 0U].r.flags |= CT_BUS_ERROR;
    /* TX notifications remain disabled. Inspect and consume completed slots
     * before EVERY AddTxMessage, including non-yaw traffic, so a reused mailbox
     * cannot be attributed to the previous command. No CAN IRQ priorities change. */
    for (i = 0U; i < 3U; ++i) {
        uint32_t bits = (tsr >> (8U*i)) & 15U;
        if (bits & 1U) {
            CanTrace_Complete(&s_trace, base+i, (bits & 2U) ? 0U : (bits & 12U) ? 1U : 2U, now);
            __HAL_CAN_CLEAR_FLAG(bus, CAN_FLAG_RQCP0 + 8U*i);
        }
    }
}
void Can_TraceBegin(void) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_cycles_per_us = SystemCoreClock / 1000000U;
    if (!s_cycles_per_us) s_cycles_per_us = 1U;
    s_trace_cycles = DWT->CYCCNT; s_trace_us = s_trace_remainder = 0U;
    CanTrace_Begin(&s_trace);
    __set_PRIMASK(mask);
}
void Can_TraceEnd(void) {
    uint32_t mask = __get_PRIMASK(); __disable_irq(); s_trace.active = 0U; __set_PRIMASK(mask);
}
uint32_t Can_TraceSnapshot(uint32_t *now_us, CanTrace_AxisRecord out[2]) {
    uint32_t mask = __get_PRIMASK(), duration; __disable_irq();
    *now_us = Can_TraceNow();
    Can_TracePollBus(&hcan1, 0U, *now_us); Can_TracePollBus(&hcan2, 3U, *now_us);
    duration = CanTrace_Snapshot(&s_trace, *now_us, out);
    __set_PRIMASK(mask); return duration;
}


/**
 * @brief        : CAN Error handle handling
 * @param         [uint32_t] ret
 * @return        [type]
 */
void Can_ErrorHandler(uint32_t ret) {
    (void)ret;
    //Log_DebugPrintf("Error: CAN Error!\n");
    while (1) {
        return;
    }
}


/**
 * @brief        : Initialize can transmitter
 * @param         [CAN_TxHeaderTypeDef] *pheader
 * @param         [uint32_t] stdid
 * @param         [uint32_t] extid
 * @param         [uint32_t] dlc
 * @return        [type]
 */
void Can_InitTxHeader(CAN_TxHeaderTypeDef *pheader, uint32_t stdid, uint32_t extid, uint32_t dlc) {
    pheader->StdId = stdid;
    pheader->ExtId = extid;
    pheader->RTR = CAN_RTR_DATA;
    pheader->IDE = CAN_ID_STD;
    pheader->DLC = dlc;
    pheader->TransmitGlobalTime = DISABLE;
}


/**
 * @brief        : Initialize can filter and enable CAN Bus Transceiver
 * @param         [CAN_HandleTypeDef*] phcan
 * @return        [type]
 */
void Can_InitFilterAndStart(CAN_HandleTypeDef* phcan) {
    CAN_FilterTypeDef sFilterConfig;

    if (phcan == &hcan1)
        sFilterConfig.FilterBank = 0;
    else
        sFilterConfig.FilterBank = 14;
    
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    uint32_t ret = HAL_CAN_ConfigFilter(phcan, &sFilterConfig);
    if (ret != HAL_OK) {
        Can_ErrorHandler(ret);
    }
    
    ret = HAL_CAN_Start(phcan);
    if (ret != HAL_OK) {
        Can_ErrorHandler(ret);
    }
    
    ret = HAL_CAN_ActivateNotification(phcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (ret != HAL_OK) {
        Can_ErrorHandler(ret);
    }
    
}


/**
 * @brief        : Sending information to can bus
 * @param         [CAN_HandleTypeDef*] phcan
 * @param         [CAN_TxHeaderTypeDef*] pheader
 * @param         [uint8_t] txdata
 * @return        [type]
 */
static uint32_t Can_Transmit(CAN_HandleTypeDef* phcan, CAN_TxHeaderTypeDef* pheader, uint8_t txdata[], uint8_t wait) {
    uint32_t mailbox = 0U;
    uint32_t timeout = 0;
    uint32_t sequence = 0U, began = 0U, ret;
    unsigned axis = 2U;
    if (s_trace.active && pheader->IDE == CAN_ID_STD && pheader->RTR == CAN_RTR_DATA && pheader->DLC == 8U) {
        uint32_t mask = __get_PRIMASK(); __disable_irq();
        if (phcan == &hcan1 && pheader->StdId == 0x2ffU) axis = 0U;
        if (phcan == &hcan2 && pheader->StdId == 0x1ffU) axis = 1U;
        began = Can_TraceNow();
        if (axis < 2U) {
            unsigned offset = axis == 0U ? 0U : 2U;
            sequence = CanTrace_Attempt(&s_trace, axis,
                (int16_t)((txdata[offset] << 8) | txdata[offset+1U]), began);
        }
        __set_PRIMASK(mask);
    }
    
    /* 防止高负载时因为发送邮箱满而直接丢弃指令，提供宽限等待 */
    while(wait && HAL_CAN_GetTxMailboxesFreeLevel(phcan) == 0 && timeout < 5000) {
        timeout++;
    }

    /* Start the Transmission process */
    if (s_trace.active) {
        uint32_t mask = __get_PRIMASK(), now; unsigned base, slot;
        __disable_irq(); now = Can_TraceNow(); base = phcan == &hcan1 ? 0U : 3U;
        Can_TracePollBus(phcan, base, now);
        ret = HAL_CAN_AddTxMessage(phcan, pheader, txdata, &mailbox);
        slot = mailbox == CAN_TX_MAILBOX0 ? 0U : mailbox == CAN_TX_MAILBOX1 ? 1U : 2U;
        CanTrace_Queue(&s_trace, axis, sequence, base+slot, (uint8_t)(ret == HAL_OK), now-began,
            (uint8_t)(timeout == 5000U));
        __set_PRIMASK(mask);
    } else ret = HAL_CAN_AddTxMessage(phcan, pheader, txdata, &mailbox);
    return ret;
}

/* The DM/Pitch path keeps its original single immediate enqueue attempt, while
 * sharing completion polling with yaw before it can reuse a hardware mailbox. */
uint32_t Can_SendMessageNoWait(CAN_HandleTypeDef* phcan, CAN_TxHeaderTypeDef* pheader, uint8_t txdata[]) {
    return Can_Transmit(phcan, pheader, txdata, 0U);
}

void Can_SendMessage(CAN_HandleTypeDef* phcan, CAN_TxHeaderTypeDef* pheader, uint8_t txdata[]) {
    uint32_t ret = Can_Transmit(phcan, pheader, txdata, 1U);
    if (ret != HAL_OK) {
        /* Transmission request Error */
        Can_ErrorHandler(ret);
    }
}


void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *phcan) {
		/* Get RX message */
    uint32_t ret = HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO1, &Can_RxHeader, Can_RxData);
    if (ret != HAL_OK) {
        /* Reception Error */
        Can_ErrorHandler(ret);
    }
    Can_RxMessageCallback(phcan, &Can_RxHeader, Can_RxData);
}


/**
 * @brief        : HAL_CAN_ Rx Fifo0 Message Pending Call back
 * @param         [CAN_HandleTypeDef] *phcan
 * @return        [type]
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *phcan) {
		/* Get RX message */
    uint32_t ret = HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &Can_RxHeader, Can_RxData);
    if (ret != HAL_OK) {
        /* Reception Error */
        Can_ErrorHandler(ret);
    }
    Can_RxMessageCallback(phcan, &Can_RxHeader, Can_RxData);		
}


/**
 * @brief        : Can bus data receiving callback function that updates the motor status according to the received information
 * @param         [CAN_HandleTypeDef*] phcan
 * @param         [CAN_RxHeaderTypeDef*] rxheader
 * @param         [uint8_t] rxdata
 * @return        [type]
 */
void Can_RxMessageCallback(CAN_HandleTypeDef* phcan, CAN_RxHeaderTypeDef* rxheader, uint8_t rxdata[]) {
    if (s_trace.active && rxheader->IDE == CAN_ID_STD && rxheader->RTR == CAN_RTR_DATA && rxheader->DLC == 8U) {
        uint32_t mask = __get_PRIMASK(); unsigned axis = 2U; __disable_irq();
        if (phcan == &hcan1 && rxheader->StdId == 0x209U) axis = 0U;
        if (phcan == &hcan2 && rxheader->StdId == 0x206U) axis = 1U;
        if (axis < 2U) CanTrace_Feedback(&s_trace, axis, rxdata, Can_TraceNow());
        __set_PRIMASK(mask);
    }
    Comm_BoardLinkRxCanFrame(rxheader->StdId, rxdata, rxheader->DLC);

    RobotActuators_DecodeCan(phcan, rxheader->StdId, rxdata, rxheader->DLC);
}
