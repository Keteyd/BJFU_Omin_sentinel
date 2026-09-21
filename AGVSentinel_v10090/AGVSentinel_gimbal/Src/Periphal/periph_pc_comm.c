/*
 * Project      : Infantry_Neptune
 * File         : periph_pc_comm.c
 * Description  : Fixed-length PC frames multiplexed on the USART1 vision link.
 */

#include "periph_pc_comm.h"
#include "app_attitude_link.h"
#include "app_yaw_identification.h"
#include "module_can_trace.h"
#include "periph_referee.h"
#include "periph_remote.h"
#include "util_uart.h"
#include <string.h>
#include <math.h>

UART_HandleTypeDef* Const_PC_Comm_UART_HANDLER = &huart1;

uint8_t PC_Comm_RxData[PC_COMM_RX_BUFF_LEN];
PC_Comm_DataTypeDef PC_Comm_Data;
PC_ParsedData_t PC_Parsed;
PC_Recv_GimbalPidTune_t PC_GimbalPidTune;
PC_Recv_GimbalPidAction_t PC_GimbalPidAction;
PC_Recv_PitchPidTune_t PC_PitchPidTune;
PC_Recv_PitchPidAction_t PC_PitchPidAction;
PC_Recv_BigYawTune_t PC_BigYawTune;
float PC_BigYawFullValues[BIG_YAW_FULL_COUNT];
static BigYaw_FullStage s_big_yaw_full_stage;
_Static_assert(sizeof(float) == 4, "Full PID wire format requires float32");

uint8_t PC_Comm_SendTrace(uint32_t trial_id, uint16_t sequence, const void *record) {
    uint8_t frame[CAN_TRACE_FRAME_BYTES];
    if (record == NULL) return 0U;
    CanTrace_Frame(frame, trial_id, sequence, record);
    return AttitudeLink_SendLegacy(frame, sizeof(frame));
}

_Static_assert(sizeof(PC_Recv_BigYawTune_t) == 12, "Big yaw tune payload size");
_Static_assert(sizeof(PC_Send_BigYawConfig_t) == 12, "Big yaw readback payload size");

uint8_t PC_Comm_IsRemoteSafe(void) {
    const Remote_RemoteDataTypeDef *remote = Remote_GetRemoteDataPtr();
    return (uint8_t)(remote->state == Remote_STATE_CONNECTED &&
        (HAL_GetTick() - remote->last_update_time) <= Const_Remote_REMOTE_OFFLINE_TIME &&
        remote->remote.s[1] == Remote_SWITCH_UP);
}

_Static_assert(sizeof(PC_Recv_Gimbal_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC gimbal payload must be 12 bytes");
_Static_assert(sizeof(PC_Recv_Chassis_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC chassis payload must be 12 bytes");
_Static_assert(sizeof(PC_Recv_Navigation_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC navigation payload must be 12 bytes");
_Static_assert(sizeof(PC_Recv_GimbalPidTune_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC gimbal PID tune payload must be 12 bytes");
_Static_assert(sizeof(PC_Recv_GimbalPidAction_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC gimbal PID action payload must be 12 bytes");
_Static_assert(sizeof(PC_Send_GimbalPidData_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC gimbal PID data payload must be 12 bytes");
_Static_assert(sizeof(PC_Send_GimbalPidAux_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC gimbal PID auxiliary payload must be 12 bytes");
_Static_assert(sizeof(PC_Recv_PitchPidTune_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC pitch PID tune payload must be 12 bytes");
_Static_assert(sizeof(PC_Recv_PitchPidAction_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC pitch PID action payload must be 12 bytes");
_Static_assert(sizeof(PC_Send_PitchPidData_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC pitch PID data payload must be 12 bytes");
_Static_assert(sizeof(PC_Send_PitchPidAux_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC pitch PID auxiliary payload must be 12 bytes");
_Static_assert(sizeof(PC_Send_PitchPidLimits_t) == PC_COMM_CMD_PAYLOAD_LEN,
               "PC pitch limits payload must be 12 bytes");

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
    memset(&PC_GimbalPidTune, 0, sizeof(PC_GimbalPidTune));
    memset(&PC_GimbalPidAction, 0, sizeof(PC_GimbalPidAction));
    memset(&PC_PitchPidTune, 0, sizeof(PC_PitchPidTune));
    memset(&PC_PitchPidAction, 0, sizeof(PC_PitchPidAction));
    memset(&PC_BigYawTune, 0, sizeof(PC_BigYawTune));
    memset(&s_big_yaw_full_stage, 0, sizeof(s_big_yaw_full_stage));
    memset(PC_BigYawFullValues, 0, sizeof(PC_BigYawFullValues));
}

void PC_Comm_Init(void) {
    PC_Comm_ResetData();
    PC_Comm_Data.state = PC_COMM_STATE_NULL;
    PC_Comm_Data.last_update_time = HAL_GetTick();

    /* USART1 RX DMA is initialized and owned by the vision/PC frame mux. */
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

    if (buff[1] == YAW_IDENT_CMD_REQUEST) {
        YawIdentApp_Receive(&buff[2]);
        return; /* Never renew legacy tune or motion deadlines. */
    }
    if (YawIdentApp_OwnsControl()) return; /* Freeze configuration and commands. */

    if (buff[1] == PC_CMD_NAVIGATION) {
        PC_Recv_Navigation_t navigation;
        memcpy(&navigation, &buff[2], sizeof(navigation));
        if (!isfinite(navigation.linear_x) || !isfinite(navigation.linear_y) ||
            !isfinite(navigation.angular_z)) {
            PC_Comm_Data.state = PC_COMM_STATE_ERROR;
            return;
        }
    }
    memcpy(PC_Comm_Data.raw_data_payload, &buff[1], PC_COMM_DATA_LEN);

    cmd_id = PC_Comm_Data.raw_data_payload[0];
    payload = &PC_Comm_Data.raw_data_payload[1];
    PC_Comm_Data.last_cmd_id = cmd_id;
    control_index = PC_Comm_ControlCommandIndex(cmd_id);

    if (cmd_id == PC_CMD_GIMBAL_PID_TUNE || cmd_id == PC_CMD_GIMBAL_PID_ACTION ||
        cmd_id == PC_CMD_PITCH_PID_TUNE || cmd_id == PC_CMD_PITCH_PID_ACTION) {
        PC_Comm_Data.tune_monitor_only = 0U;
    }

    if (cmd_id == PC_CMD_GIMBAL_PID_TUNE ||
        cmd_id == PC_CMD_GIMBAL_PID_ACTION ||
        cmd_id == PC_CMD_PITCH_PID_TUNE ||
        cmd_id == PC_CMD_PITCH_PID_ACTION ||
        cmd_id == PC_CMD_BIG_YAW_TUNE ||
        cmd_id == PC_CMD_BIG_YAW_FULL_TUNE ||
        cmd_id == PC_CMD_GIMBAL_PID_SESSION) {
        PC_Comm_Data.gimbal_tune_session_seen = 1U;
        PC_Comm_Data.gimbal_tune_session_time = HAL_GetTick();
    }

    /* Tuning traffic must not make a stale motion command appear online. */
    if (cmd_id != PC_CMD_GIMBAL_PID_TUNE &&
        cmd_id != PC_CMD_GIMBAL_PID_ACTION &&
        cmd_id != PC_CMD_PITCH_PID_TUNE &&
        cmd_id != PC_CMD_PITCH_PID_ACTION &&
        cmd_id != PC_CMD_BIG_YAW_TUNE &&
        cmd_id != PC_CMD_BIG_YAW_FULL_TUNE &&
        cmd_id != PC_CMD_GIMBAL_PID_SESSION) {
        PC_Comm_Data.state = PC_COMM_STATE_CONNECTED;
        PC_Comm_Data.last_update_time = HAL_GetTick();
    }

    switch (cmd_id) {
        case PC_CMD_BIG_YAW_FULL_TUNE: {
            uint32_t now = HAL_GetTick();
            uint8_t safe = PC_Comm_IsRemoteSafe();
            uint8_t result = BigYaw_FullReceive(&s_big_yaw_full_stage, payload, now, safe);
            PC_Comm_Data.active_tune_axis = PC_TUNE_AXIS_SMALL_YAW;
            PC_Comm_Data.tune_monitor_only = 1U;
            if (result != 0U) {
                if (result == 1U)
                    memcpy(PC_BigYawFullValues, s_big_yaw_full_stage.values, sizeof(PC_BigYawFullValues));
                PC_Comm_Data.big_yaw_full_id = (uint16_t)(payload[0] | ((uint16_t)payload[1] << 8));
                PC_Comm_Data.big_yaw_full_status = result;
                PC_Comm_Data.big_yaw_tune_full = 1U;
                PC_Comm_Data.big_yaw_tune_rx_safe = safe;
                PC_Comm_Data.big_yaw_tune_rx_time = now;
                ++PC_Comm_Data.big_yaw_tune_sequence;
            }
            break;
        }
        case PC_CMD_BIG_YAW_TUNE:
            s_big_yaw_full_stage.active = 0U;
            PC_Comm_Data.big_yaw_tune_full = 0U;
            PC_Comm_CopyPayload(&PC_BigYawTune, sizeof(PC_BigYawTune),
                                payload, PC_COMM_CMD_PAYLOAD_LEN);
            PC_Comm_Data.big_yaw_tune_rx_safe = PC_Comm_IsRemoteSafe();
            PC_Comm_Data.big_yaw_tune_rx_time = HAL_GetTick();
            ++PC_Comm_Data.big_yaw_tune_sequence;
            PC_Comm_Data.active_tune_axis = PC_TUNE_AXIS_SMALL_YAW;
            PC_Comm_Data.tune_monitor_only = 1U;
            break;
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

        case PC_CMD_GIMBAL_PID_TUNE:
            PC_Comm_CopyPayload(&PC_GimbalPidTune, sizeof(PC_GimbalPidTune),
                                payload, PC_COMM_CMD_PAYLOAD_LEN);
            ++PC_Comm_Data.gimbal_tune_sequence;
            PC_Comm_Data.active_tune_axis = PC_TUNE_AXIS_SMALL_YAW;
            break;

        case PC_CMD_GIMBAL_PID_ACTION:
            PC_Comm_CopyPayload(&PC_GimbalPidAction, sizeof(PC_GimbalPidAction),
                                payload, PC_COMM_CMD_PAYLOAD_LEN);
            ++PC_Comm_Data.gimbal_tune_action_sequence;
            PC_Comm_Data.active_tune_axis = PC_TUNE_AXIS_SMALL_YAW;
            break;

        case PC_CMD_PITCH_PID_TUNE:
            PC_Comm_CopyPayload(&PC_PitchPidTune, sizeof(PC_PitchPidTune),
                                payload, PC_COMM_CMD_PAYLOAD_LEN);
            ++PC_Comm_Data.pitch_tune_sequence;
            PC_Comm_Data.active_tune_axis = PC_TUNE_AXIS_PITCH;
            break;

        case PC_CMD_PITCH_PID_ACTION:
            PC_Comm_CopyPayload(&PC_PitchPidAction, sizeof(PC_PitchPidAction),
                                payload, PC_COMM_CMD_PAYLOAD_LEN);
            ++PC_Comm_Data.pitch_tune_action_sequence;
            PC_Comm_Data.active_tune_axis = PC_TUNE_AXIS_PITCH;
            break;

        case PC_CMD_GIMBAL_PID_SESSION:
            PC_Comm_Data.yaw_capture_requested = (uint8_t)(
                payload[0] == PC_TUNE_SESSION_MONITOR &&
                payload[1] == PC_TUNE_AXIS_SMALL_YAW && payload[2] == 0xD1U &&
                payload[3] == 1U);
            if (payload[0] == PC_TUNE_SESSION_MONITOR &&
                payload[1] == PC_TUNE_AXIS_SMALL_YAW &&
                payload[2] == 0xD2U && payload[3] == 2U)
                PC_Comm_Data.yaw_capture_requested = 2U;
            if (payload[0] == PC_TUNE_SESSION_MONITOR &&
                (payload[1] == PC_TUNE_AXIS_SMALL_YAW ||
                 payload[1] == PC_TUNE_AXIS_PITCH)) {
                PC_Comm_Data.active_tune_axis = payload[1];
                PC_Comm_Data.tune_monitor_only = 1U;
            } else {
                PC_Comm_Data.tune_monitor_only = 0U;
            }
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

uint32_t PC_Comm_GetGimbalTuneSequence(void) {
    return PC_Comm_Data.gimbal_tune_sequence;
}

uint32_t PC_Comm_GetGimbalTuneActionSequence(void) {
    return PC_Comm_Data.gimbal_tune_action_sequence;
}

uint32_t PC_Comm_GetPitchTuneSequence(void) {
    return PC_Comm_Data.pitch_tune_sequence;
}

uint32_t PC_Comm_GetPitchTuneActionSequence(void) {
    return PC_Comm_Data.pitch_tune_action_sequence;
}

uint8_t PC_Comm_GetActiveTuneAxis(void) {
    return PC_Comm_Data.active_tune_axis;
}

uint8_t PC_Comm_IsGimbalTuneSessionOnline(void) {
    return (uint8_t)(PC_Comm_Data.gimbal_tune_session_seen != 0U &&
                     (HAL_GetTick() - PC_Comm_Data.gimbal_tune_session_time) <=
                         PC_COMM_TUNE_SESSION_TIME);
}

uint8_t PC_Comm_IsGimbalTuneControlLocked(void) {
    return (uint8_t)(PC_Comm_IsGimbalTuneSessionOnline() &&
                     PC_Comm_Data.tune_monitor_only == 0U);
}

uint8_t PC_Comm_IsYawCaptureOnline(void) {
    return (uint8_t)(PC_Comm_IsGimbalTuneSessionOnline() &&
        PC_Comm_Data.tune_monitor_only != 0U &&
        PC_Comm_Data.active_tune_axis == PC_TUNE_AXIS_SMALL_YAW &&
        PC_Comm_Data.yaw_capture_requested != 0U);
}

const PC_Recv_GimbalPidTune_t* PC_Comm_GetGimbalTune(void) {
    return &PC_GimbalPidTune;
}

const PC_Recv_GimbalPidAction_t* PC_Comm_GetGimbalTuneAction(void) {
    return &PC_GimbalPidAction;
}

const PC_Recv_PitchPidTune_t* PC_Comm_GetPitchTune(void) {
    return &PC_PitchPidTune;
}

const PC_Recv_PitchPidAction_t* PC_Comm_GetPitchTuneAction(void) {
    return &PC_PitchPidAction;
}

uint8_t PC_Comm_SendPacket(uint8_t cmd_id, const void *payload, uint16_t payload_len) {
    uint8_t PC_Comm_TxData[PC_COMM_PACKET_LEN];
    uint16_t copy_len = payload_len;

    if (Const_PC_Comm_UART_HANDLER == NULL || payload == NULL) return 0U;
    if (copy_len > PC_COMM_CMD_PAYLOAD_LEN) copy_len = PC_COMM_CMD_PAYLOAD_LEN;

    memset(PC_Comm_TxData, 0, sizeof(PC_Comm_TxData));
    PC_Comm_TxData[0] = PC_COMM_HEADER_SOF;
    PC_Comm_TxData[1] = cmd_id;
    memcpy(&PC_Comm_TxData[2], payload, copy_len);
    PC_Comm_TxData[PC_COMM_CRC_OFFSET] =
        PC_Comm_CalculateCRC8(PC_Comm_TxData, PC_COMM_CRC_OFFSET);
    PC_Comm_TxData[PC_COMM_TAIL_OFFSET] = PC_COMM_TAIL_EOF;

    return AttitudeLink_SendLegacy(PC_Comm_TxData, PC_COMM_PACKET_LEN);
}

uint8_t PC_Comm_SendBatch(uint8_t cmd_id, const uint8_t *payloads, uint8_t count) {
    uint8_t PC_Comm_BatchTxData[6U * PC_COMM_PACKET_LEN];
    uint8_t i;
    if (payloads == NULL || count == 0U || count > 6U) return 0U;
    for (i = 0U; i < count; ++i) {
        uint8_t *frame = PC_Comm_BatchTxData + i * PC_COMM_PACKET_LEN;
        frame[0] = PC_COMM_HEADER_SOF; frame[1] = cmd_id;
        memcpy(frame + 2U, payloads + i * PC_COMM_CMD_PAYLOAD_LEN, PC_COMM_CMD_PAYLOAD_LEN);
        frame[PC_COMM_CRC_OFFSET] = PC_Comm_CalculateCRC8(frame, PC_COMM_CRC_OFFSET);
        frame[PC_COMM_TAIL_OFFSET] = PC_COMM_TAIL_EOF;
    }
    return AttitudeLink_SendLegacy(PC_Comm_BatchTxData,
                                  (uint16_t)(count * PC_COMM_PACKET_LEN));
}
