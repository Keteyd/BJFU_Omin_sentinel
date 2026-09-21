#include "app_communicate.h"

#include "app_autoaim.h"
#include "periph_pc_comm.h"
#include "periph_referee.h"
#include "periph_remote.h"
#include "sys_const.h"
#include "util_can.h"
#include <string.h>

#if BOARD_HAS_GIMBAL
#include "module_gimbal.h"
#endif

#define CANLINK_G2C_CHASSIS_COMMAND_ID  0x310U
#define CANLINK_G2C_HEARTBEAT_ID        0x311U
#define CANLINK_G2C_GIMBAL_FEEDBACK_ID  0x314U
#define CANLINK_C2G_HEARTBEAT_ID        0x320U
#define CANLINK_C2G_REF_BASIC_ID        0x340U
#define CANLINK_C2G_REF_LIMIT_ID        0x341U

#define CANLINK_COMMAND_TIMEOUT_MS      100U
#define CANLINK_HEARTBEAT_TIMEOUT_MS    200U
#define CANLINK_FEEDBACK_TIMEOUT_MS     200U
#define CANLINK_REFEREE_TIMEOUT_MS      1000U

static CAN_TxHeaderTypeDef s_tx_header;
static Comm_ChassisCommandTypeDef s_chassis_command;
static uint8_t s_initialized;
static uint8_t s_tx_slot;
static uint8_t s_tx_sequence;
static uint32_t s_chassis_last_update;
static uint32_t s_gimbal_last_update;
static uint32_t s_feedback_last_update;
static uint32_t s_referee_last_update;
#if BOARD_HAS_GIMBAL
static uint8_t s_referee_reported_online;
#endif
static float s_gimbal_rel_yaw_deg;
static float s_gimbal_pitch_rad;

static void Comm_PackI16(uint8_t *dst, int16_t value)
{
    dst[0] = (uint8_t)((uint16_t)value & 0xFFU);
    dst[1] = (uint8_t)(((uint16_t)value >> 8) & 0xFFU);
}

static int16_t Comm_UnpackI16(const uint8_t *src)
{
    return (int16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

static void Comm_PackU16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)(value >> 8);
}

static uint16_t Comm_UnpackU16(const uint8_t *src)
{
    return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

static void Comm_PackF32(uint8_t *dst, float value)
{
    memcpy(dst, &value, sizeof(value));
}

static float Comm_UnpackF32(const uint8_t *src)
{
    float value;
    memcpy(&value, src, sizeof(value));
    return value;
}

static uint16_t Comm_FloatToU16(float value, float scale)
{
    float scaled = value * scale;
    if (scaled <= 0.0f) return 0U;
    if (scaled >= 65535.0f) return 65535U;
    return (uint16_t)scaled;
}

static void Comm_Send(CAN_HandleTypeDef *can, uint32_t stdid, uint8_t data[8])
{
    s_tx_header.StdId = stdid;
    Can_SendMessage(can, &s_tx_header, data);
}

void Comm_BoardLinkInit(void)
{
    memset(&s_chassis_command, 0, sizeof(s_chassis_command));
    Can_InitTxHeader(&s_tx_header, 0U, 0U, 8U);
    s_initialized = 1U;
    s_tx_slot = 0U;
    s_tx_sequence = 0U;
    s_chassis_last_update = 0U;
    s_gimbal_last_update = 0U;
    s_feedback_last_update = 0U;
    s_referee_last_update = 0U;
#if BOARD_HAS_GIMBAL
    s_referee_reported_online = 0U;
#endif
}

void Comm_BoardLinkSetChassisCommand(const Comm_ChassisCommandTypeDef *command)
{
#if BOARD_HAS_GIMBAL
    if (command == NULL) return;
    s_chassis_command.vx_ref = command->vx_ref;
    s_chassis_command.vy_ref = command->vy_ref;
    s_chassis_command.wz_ref = command->wz_ref;
    s_chassis_command.mode = command->mode;
#else
    (void)command;
#endif
}

const Comm_ChassisCommandTypeDef *Comm_BoardLinkGetChassisCommand(void)
{
    return &s_chassis_command;
}

#if BOARD_HAS_GIMBAL
static void Comm_GimbalTxStep(uint8_t data[8])
{
    switch (s_tx_slot) {
        case 0U:
            Comm_PackI16(&data[0], s_chassis_command.vx_ref);
            Comm_PackI16(&data[2], s_chassis_command.vy_ref);
            Comm_PackI16(&data[4], s_chassis_command.wz_ref);
            data[6] = s_chassis_command.mode;
            data[7] = s_tx_sequence;
            Comm_Send(&hcan1, CANLINK_G2C_CHASSIS_COMMAND_ID, data);
            break;
        case 1U:
            data[0] = s_tx_sequence;
            data[1] = Comm_BoardLinkRemoteOnline();
            data[2] = PC_Comm_IsOnline();
            data[3] = AutoAim_IsOnline(100U);
            Comm_Send(&hcan1, CANLINK_G2C_HEARTBEAT_ID, data);
            break;
        default:
            Comm_PackF32(&data[0], GimbalYaw_GetEncoderRawDeg() -
                                      CHASSIS_YAW_ANGLE_OFFSET);
            Comm_PackF32(&data[4], GimbalPitch_GetPositionFeedback());
            Comm_Send(&hcan1, CANLINK_G2C_GIMBAL_FEEDBACK_ID, data);
            s_tx_sequence++;
            break;
    }
    s_tx_slot = (uint8_t)((s_tx_slot + 1U) % 3U);
}
#else
static void Comm_ChassisTxStep(uint8_t data[8])
{
    Referee_RefereeDataTypeDef *referee = Referee_GetRefereeDataPtr();

    switch (s_tx_slot) {
        case 0U:
            data[0] = s_tx_sequence;
            data[1] = Comm_BoardLinkChassisCommandOnline();
            data[2] = Comm_BoardLinkRefereeOnline();
            Comm_Send(&hcan2, CANLINK_C2G_HEARTBEAT_ID, data);
            break;
        case 1U:
            data[0] = (uint8_t)referee->game_state;
            data[1] = referee->robot_id;
            data[2] = (uint8_t)((referee->mains_power_gimbal_output ? 1U : 0U) |
                                ((referee->mains_power_chassis_output ? 1U : 0U) << 1) |
                                ((referee->mains_power_shooter_output ? 1U : 0U) << 2));
            Comm_PackU16(&data[3], Comm_FloatToU16(referee->bullet_speed, 100.0f));
            Comm_PackU16(&data[5], referee->shooter_heat0);
            data[7] = s_tx_sequence;
            Comm_Send(&hcan2, CANLINK_C2G_REF_BASIC_ID, data);
            break;
        default:
            Comm_PackU16(&data[0], referee->shooter_heat0_speed_limit);
            Comm_PackU16(&data[2], referee->shooter_heat0_cooling_limit);
            Comm_PackU16(&data[4], referee->shooter_heat1);
            Comm_PackU16(&data[6], referee->chassis_power_buffer);
            Comm_Send(&hcan2, CANLINK_C2G_REF_LIMIT_ID, data);
            s_tx_sequence++;
            break;
    }
    s_tx_slot = (uint8_t)((s_tx_slot + 1U) % 3U);
}
#endif

void Comm_BoardLinkTxStep(void)
{
    uint8_t data[8] = {0};
    if (!s_initialized) Comm_BoardLinkInit();
#if BOARD_HAS_GIMBAL
    Comm_GimbalTxStep(data);
#else
    Comm_ChassisTxStep(data);
#endif
}

void Comm_BoardLinkRxCanFrame(uint32_t stdid, const uint8_t *data, uint32_t len)
{
    uint32_t now;
    if (data == NULL || len != 8U) return;
    now = HAL_GetTick();

#if BOARD_HAS_CHASSIS
    if (stdid == CANLINK_G2C_CHASSIS_COMMAND_ID) {
        s_chassis_command.vx_ref = Comm_UnpackI16(&data[0]);
        s_chassis_command.vy_ref = Comm_UnpackI16(&data[2]);
        s_chassis_command.wz_ref = Comm_UnpackI16(&data[4]);
        s_chassis_command.mode = data[6];
        s_chassis_command.sequence = data[7];
        s_chassis_command.last_update_time = now;
        s_gimbal_last_update = now;
    } else if (stdid == CANLINK_G2C_HEARTBEAT_ID) {
        s_gimbal_last_update = now;
    } else if (stdid == CANLINK_G2C_GIMBAL_FEEDBACK_ID) {
        s_gimbal_rel_yaw_deg = Comm_UnpackF32(&data[0]);
        s_gimbal_pitch_rad = Comm_UnpackF32(&data[4]);
        s_feedback_last_update = now;
    }
#else
    Referee_RefereeDataTypeDef *referee = Referee_GetRefereeDataPtr();
    if (stdid == CANLINK_C2G_HEARTBEAT_ID) {
        s_chassis_last_update = now;
        s_referee_reported_online = (uint8_t)(data[2] != 0U);
        if (!s_referee_reported_online) {
            referee->state = Referee_STATE_LOST;
        }
    } else if (stdid == CANLINK_C2G_REF_BASIC_ID) {
        referee->game_state = data[0];
        referee->game_progress = data[0];
        referee->robot_id = data[1];
        referee->client_id = Referee_GetClientIDByRobotID(referee->robot_id);
        referee->mains_power_gimbal_output = (data[2] & 0x01U) != 0U;
        referee->mains_power_chassis_output = (data[2] & 0x02U) != 0U;
        referee->mains_power_shooter_output = (data[2] & 0x04U) != 0U;
        referee->bullet_speed = (float)Comm_UnpackU16(&data[3]) * 0.01f;
        referee->shooter_heat0 = Comm_UnpackU16(&data[5]);
        referee->state = s_referee_reported_online ?
                             Referee_STATE_CONNECTED : Referee_STATE_LOST;
        referee->last_update_time = now;
        s_referee_last_update = now;
    } else if (stdid == CANLINK_C2G_REF_LIMIT_ID) {
        referee->shooter_heat0_speed_limit = Comm_UnpackU16(&data[0]);
        referee->shooter_heat0_cooling_limit = Comm_UnpackU16(&data[2]);
        referee->shooter_heat1 = Comm_UnpackU16(&data[4]);
        referee->chassis_power_buffer = Comm_UnpackU16(&data[6]);
        referee->state = s_referee_reported_online ?
                             Referee_STATE_CONNECTED : Referee_STATE_LOST;
        referee->last_update_time = now;
        s_referee_last_update = now;
    }
#endif
}

uint8_t Comm_BoardLinkRemoteOnline(void)
{
#if BOARD_HAS_GIMBAL
    Remote_RemoteDataTypeDef *remote = Remote_GetRemoteDataPtr();
    return (uint8_t)(remote->state == Remote_STATE_CONNECTED && !Remote_IsRemoteOffline());
#else
    return 0U;
#endif
}

uint8_t Comm_BoardLinkChassisOnline(void)
{
#if BOARD_HAS_GIMBAL
    return (uint8_t)(s_chassis_last_update != 0U &&
                     (HAL_GetTick() - s_chassis_last_update) <= CANLINK_HEARTBEAT_TIMEOUT_MS);
#else
    return 1U;
#endif
}

uint8_t Comm_BoardLinkChassisCommandOnline(void)
{
#if BOARD_HAS_CHASSIS
    return (uint8_t)(s_chassis_command.last_update_time != 0U &&
                     (HAL_GetTick() - s_chassis_command.last_update_time) <=
                         CANLINK_COMMAND_TIMEOUT_MS &&
                     s_gimbal_last_update != 0U &&
                     (HAL_GetTick() - s_gimbal_last_update) <=
                         CANLINK_HEARTBEAT_TIMEOUT_MS);
#else
    return 1U;
#endif
}

uint8_t Comm_BoardLinkGimbalFeedbackOnline(void)
{
#if BOARD_HAS_CHASSIS
    return (uint8_t)(s_feedback_last_update != 0U &&
                     (HAL_GetTick() - s_feedback_last_update) <=
                         CANLINK_FEEDBACK_TIMEOUT_MS);
#else
    return 1U;
#endif
}

uint8_t Comm_BoardLinkRefereeOnline(void)
{
#if BOARD_HAS_GIMBAL
    return (uint8_t)(s_referee_reported_online &&
                     s_chassis_last_update != 0U &&
                     (HAL_GetTick() - s_chassis_last_update) <=
                         CANLINK_HEARTBEAT_TIMEOUT_MS &&
                     s_referee_last_update != 0U &&
                     (HAL_GetTick() - s_referee_last_update) <=
                         CANLINK_REFEREE_TIMEOUT_MS);
#else
    Referee_RefereeDataTypeDef *referee = Referee_GetRefereeDataPtr();
    return (uint8_t)(referee->state == Referee_STATE_CONNECTED &&
                     !Referee_IsRefereeOffline());
#endif
}

float Comm_BoardLinkGimbalRelYawDeg(void)
{
#if BOARD_HAS_GIMBAL
    return (GimbalYaw_GetEncoderRawDeg() - CHASSIS_YAW_ANGLE_OFFSET) +
           CHASSIS_GIMBAL_YAW_STEER_OFFSET;
#else
    return s_gimbal_rel_yaw_deg + CHASSIS_GIMBAL_YAW_STEER_OFFSET;
#endif
}

float Comm_BoardLinkGimbalPitchRad(void)
{
#if BOARD_HAS_GIMBAL
    return GimbalPitch_GetPositionFeedback();
#else
    return s_gimbal_pitch_rad;
#endif
}

void Comm_Task(void const *argument)
{
    (void)argument;
    for (;;) {
        Comm_BoardLinkTxStep();
        osDelay(1U);
    }
}
