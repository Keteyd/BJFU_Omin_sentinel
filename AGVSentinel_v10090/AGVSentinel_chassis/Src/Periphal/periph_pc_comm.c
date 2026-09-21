/*
 * Project      : Infantry_Neptune
 * File         : periph_pc_comm.c
 * Description  : Fixed-length UART6 PC communication frame parser.
 */

#include "periph_pc_comm.h"
#include "periph_referee.h"
#include "util_uart.h"
#include <string.h>

UART_HandleTypeDef* Const_PC_Comm_UART_HANDLER = &huart6;

uint8_t PC_Comm_RxData[PC_COMM_RX_BUFF_LEN];
PC_Comm_DataTypeDef PC_Comm_Data;
PC_ParsedData_t PC_Parsed;

_Static_assert(sizeof(PC_Recv_Gimbal_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC gimbal payload must be 12 bytes");
_Static_assert(sizeof(PC_Recv_Chassis_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC chassis payload must be 12 bytes");
_Static_assert(sizeof(PC_Recv_Navigation_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC navigation payload must be 12 bytes");

static int8_t PC_Comm_ControlCommandIndex(uint8_t cmd_id) {
    if (cmd_id == PC_CMD_GIMBAL) return 0;
    if (cmd_id == PC_CMD_CHASSIS) return 1;
    if (cmd_id == PC_CMD_NAVIGATION) return 2;
    return -1;
}

static void PC_Comm_CopyPayload(void *dst, uint16_t dst_size, const uint8_t *payload, uint16_t payload_len) {
    uint16_t copy_len = dst_size;

    if (copy_len > payload_len) {
        copy_len = payload_len;
    }

    memset(dst, 0, dst_size);
    memcpy(dst, payload, copy_len);
}

void PC_Comm_ResetData(void) {
    memset(&PC_Comm_Data, 0, sizeof(PC_Comm_Data));
    memset(&PC_Parsed, 0, sizeof(PC_Parsed));
}

void PC_Comm_Init(void) {
    PC_Comm_ResetData();
    PC_Comm_Data.state = PC_COMM_STATE_NULL;
    PC_Comm_Data.last_update_time = HAL_GetTick();

    Uart_InitUartDMA(Const_PC_Comm_UART_HANDLER);
    Uart_ReceiveDMA(Const_PC_Comm_UART_HANDLER, PC_Comm_RxData, PC_COMM_RX_BUFF_LEN);
}

PC_Comm_StateEnum PC_Comm_CheckTimeout(void) {
    uint32_t now = HAL_GetTick();

    if (PC_Comm_Data.state == PC_COMM_STATE_CONNECTED &&
        (now - PC_Comm_Data.last_update_time) > PC_COMM_OFFLINE_TIME) {
        PC_Comm_Data.state = PC_COMM_STATE_LOST;
    }

    return PC_Comm_Data.state;
}

uint8_t PC_Comm_CalculateCRC8(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00U;

    if (data == NULL) {
        return 0U;
    }

    for (uint16_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ 0x31U);
            } else {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }

    return crc;
}

uint8_t PC_Comm_VerifyChecksum(const uint8_t *buff) {
    uint8_t expected_crc;

    if (buff == NULL) {
        return 0U;
    }

    expected_crc = PC_Comm_CalculateCRC8(buff, PC_COMM_PACKET_LEN - 2U);
    return expected_crc == buff[PC_COMM_CRC_OFFSET];
}

void PC_Comm_RXCallback(UART_HandleTypeDef* huart) {
    uint16_t rxdatalen = 0U;
    uint16_t remaining;

    if (huart == NULL || Const_PC_Comm_UART_HANDLER == NULL || huart->hdmarx == NULL) {
        return;
    }

    if (huart->Instance != Const_PC_Comm_UART_HANDLER->Instance) {
        return;
    }

    __HAL_DMA_DISABLE(huart->hdmarx);

    remaining = Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
    if (remaining <= PC_COMM_RX_BUFF_LEN) {
        rxdatalen = PC_COMM_RX_BUFF_LEN - remaining;
    }

    if (rxdatalen >= PC_COMM_PACKET_LEN) {
        for (uint16_t i = 0U; (uint16_t)(i + PC_COMM_PACKET_LEN) <= rxdatalen; ++i) {
            if (PC_Comm_RxData[i] == PC_COMM_HEADER_SOF &&
                PC_Comm_RxData[i + PC_COMM_TAIL_OFFSET] == PC_COMM_TAIL_EOF) {
                PC_Comm_DecodePacket(&PC_Comm_RxData[i], PC_COMM_PACKET_LEN);
                i = (uint16_t)(i + PC_COMM_PACKET_LEN - 1U);
            }
        }
    }

    __HAL_DMA_SET_COUNTER(huart->hdmarx, PC_COMM_RX_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

void PC_Comm_DecodePacket(const uint8_t *buff, uint16_t rxdatalen) {
    const uint8_t *payload;
    uint8_t cmd_id;
    int8_t control_index;

    if (buff == NULL || rxdatalen < PC_COMM_PACKET_LEN) {
        return;
    }

    if (buff[0] != PC_COMM_HEADER_SOF || buff[PC_COMM_TAIL_OFFSET] != PC_COMM_TAIL_EOF) {
        return;
    }

    if (!PC_Comm_VerifyChecksum(buff)) {
        PC_Comm_Data.state = PC_COMM_STATE_ERROR;
        return;
    }

    PC_Comm_Data.state = PC_COMM_STATE_CONNECTED;
    PC_Comm_Data.last_update_time = HAL_GetTick();
    memcpy(PC_Comm_Data.raw_data_payload, &buff[1], PC_COMM_DATA_LEN);

    cmd_id = PC_Comm_Data.raw_data_payload[0];
    payload = &PC_Comm_Data.raw_data_payload[1];
    PC_Comm_Data.last_cmd_id = cmd_id;
    control_index = PC_Comm_ControlCommandIndex(cmd_id);

    switch (cmd_id) {
        case PC_CMD_GIMBAL:
            PC_Comm_CopyPayload(&PC_Parsed.control.gimbal, sizeof(PC_Parsed.control.gimbal),
                                payload, PC_COMM_CMD_PAYLOAD_LEN);
            break;

        case PC_CMD_CHASSIS:
            PC_Comm_CopyPayload(&PC_Parsed.control.chassis, sizeof(PC_Parsed.control.chassis),
                                payload, PC_COMM_CMD_PAYLOAD_LEN);
            break;

        case PC_CMD_NAVIGATION:
            PC_Comm_CopyPayload(&PC_Parsed.control.navigation, sizeof(PC_Parsed.control.navigation),
                                payload, PC_COMM_CMD_PAYLOAD_LEN);
            break;

        case PC_CMD_POWER_HEAT:
            PC_Comm_CopyPayload(&PC_Parsed.referee.power_heat, sizeof(PC_Parsed.referee.power_heat),
                                payload, PC_COMM_CMD_PAYLOAD_LEN);
            break;

        case PC_CMD_DAMAGE_HEALTH:
            PC_Comm_CopyPayload(&PC_Parsed.referee.damage, sizeof(PC_Parsed.referee.damage),
                                payload, PC_COMM_CMD_PAYLOAD_LEN);
            break;

        case PC_CMD_SHOOT_STATUS:
            PC_Comm_CopyPayload(&PC_Parsed.referee.shoot, sizeof(PC_Parsed.referee.shoot),
                                payload, PC_COMM_CMD_PAYLOAD_LEN);
            break;

        case PC_CMD_FIELD_BUFF:
            PC_Comm_CopyPayload(&PC_Parsed.referee.field_buff, sizeof(PC_Parsed.referee.field_buff),
                                payload, PC_COMM_CMD_PAYLOAD_LEN);
            break;

        default:
            break;
    }

    if (control_index >= 0) {
        PC_Comm_Data.control_update_time[(uint8_t)control_index] = PC_Comm_Data.last_update_time;
        PC_Comm_Data.control_seen_mask |= (uint8_t)(1U << (uint8_t)control_index);
    }
}

uint8_t PC_Comm_IsOnline(void) {
    return PC_Comm_CheckTimeout() == PC_COMM_STATE_CONNECTED;
}

PC_Comm_StateEnum PC_Comm_GetState(void) {
    return PC_Comm_CheckTimeout();
}

PC_Comm_DataTypeDef* PC_Comm_GetDataPtr(void) {
    PC_Comm_CheckTimeout();
    return &PC_Comm_Data;
}

PC_ParsedData_t* PC_Comm_GetParsedDataPtr(void) {
    PC_Comm_CheckTimeout();
    return &PC_Parsed;
}

uint8_t PC_Comm_IsCommandOnline(uint8_t cmd_id) {
    int8_t index = PC_Comm_ControlCommandIndex(cmd_id);

    if (index < 0) return 0U;
    if ((PC_Comm_Data.control_seen_mask & (uint8_t)(1U << (uint8_t)index)) == 0U) return 0U;
    return (uint8_t)((HAL_GetTick() - PC_Comm_Data.control_update_time[(uint8_t)index]) <=
                     PC_COMM_OFFLINE_TIME);
}

uint32_t PC_Comm_GetCommandLastUpdate(uint8_t cmd_id) {
    int8_t index = PC_Comm_ControlCommandIndex(cmd_id);
    if (index < 0) return 0U;
    return PC_Comm_Data.control_update_time[(uint8_t)index];
}
