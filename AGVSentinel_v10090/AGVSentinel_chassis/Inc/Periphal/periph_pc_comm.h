/*
 * Project      : Infantry_Neptune
 * File         : periph_pc_comm.h
 * Description  : Fixed-length UART6 PC communication frame parser.
 */

#ifndef PERIPH_PC_COMM_H
#define PERIPH_PC_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include <stdint.h>

#define PC_COMM_PACKET_LEN          16U
#define PC_COMM_HEADER_SOF          0xFFU
#define PC_COMM_TAIL_EOF            0x0DU
#define PC_COMM_DATA_LEN            13U
#define PC_COMM_CMD_PAYLOAD_LEN     (PC_COMM_DATA_LEN - 1U)
#define PC_COMM_RX_BUFF_LEN         64U
#define PC_COMM_CRC_OFFSET          14U
#define PC_COMM_TAIL_OFFSET         15U
#define PC_COMM_OFFLINE_TIME        100U

extern UART_HandleTypeDef* Const_PC_Comm_UART_HANDLER;

typedef enum {
    PC_COMM_STATE_NULL      = 0,
    PC_COMM_STATE_CONNECTED = 1,
    PC_COMM_STATE_LOST      = 2,
    PC_COMM_STATE_ERROR     = 3
} PC_Comm_StateEnum;

typedef struct {
    PC_Comm_StateEnum state;
    uint32_t last_update_time;
    uint8_t raw_data_payload[PC_COMM_DATA_LEN];
    uint8_t last_cmd_id;
    uint8_t control_seen_mask;
    uint32_t control_update_time[3];
} PC_Comm_DataTypeDef;

typedef enum {
    PC_CMD_GIMBAL          = 0x01,
    PC_CMD_CHASSIS         = 0x02,
    PC_CMD_NAVIGATION      = 0x03,
    PC_CMD_POWER_HEAT      = 0x10,
    PC_CMD_DAMAGE_HEALTH   = 0x11,
    PC_CMD_SHOOT_STATUS    = 0x12,
    PC_CMD_FIELD_BUFF      = 0x13
} PC_Comm_CmdID_e;

#define PC_GIMBAL_MODE_ENABLE  (1U << 0)
#define PC_GIMBAL_MODE_AUTOAIM (1U << 1)

#pragma pack(push, 1)

typedef struct {
    int16_t yaw_rate_cdeg_s;
    int16_t pitch_rate_mrad_s;
    uint8_t fire;
    uint8_t mode;
    uint8_t reserved[6];
} PC_Recv_Gimbal_t;

typedef struct {
    int16_t vx_mm_s;
    int16_t vy_mm_s;
    int16_t wz_mrad_s;
    uint8_t mode;
    uint8_t reserved[5];
} PC_Recv_Chassis_t;

typedef struct {
    float linear_x;
    float linear_y;
    float angular_z;
} PC_Recv_Navigation_t;

typedef struct {
    uint16_t chassis_power_limit;
    uint16_t shooter_barrel_heat_limit;
    uint16_t shooter_barrel_cooling_value;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} PC_Recv_PowerHeat_t;

typedef struct {
    uint16_t current_hp;
    uint8_t armor_id : 4;
    uint8_t hp_deduction_reason : 4;
} PC_Recv_Damage_t;

typedef struct {
    uint8_t launching_frequency;
    float initial_speed;
    uint16_t allowance_17mm;
    uint16_t allowance_42mm;
} PC_Recv_Shoot_t;

typedef struct {
    uint32_t rfid_status;
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
} PC_Recv_FieldBuff_t;

typedef struct {
    PC_Recv_Gimbal_t gimbal;
    PC_Recv_Chassis_t chassis;
    PC_Recv_Navigation_t navigation;
} PC_ControlData_t;

typedef struct {
    PC_Recv_PowerHeat_t power_heat;
    PC_Recv_Damage_t damage;
    PC_Recv_Shoot_t shoot;
    PC_Recv_FieldBuff_t field_buff;
} PC_RefereeData_t;

typedef struct {
    PC_ControlData_t control;
    PC_RefereeData_t referee;
} PC_ParsedData_t;

#pragma pack(pop)

extern PC_Comm_DataTypeDef PC_Comm_Data;
extern PC_ParsedData_t PC_Parsed;

void PC_Comm_Init(void);
void PC_Comm_ResetData(void);
PC_Comm_StateEnum PC_Comm_CheckTimeout(void);
uint8_t PC_Comm_CalculateCRC8(const uint8_t *data, uint16_t len);
uint8_t PC_Comm_VerifyChecksum(const uint8_t *buff);
void PC_Comm_DecodePacket(const uint8_t *buff, uint16_t rxdatalen);
void PC_Comm_RXCallback(UART_HandleTypeDef* huart);
uint8_t PC_Comm_IsOnline(void);
PC_Comm_StateEnum PC_Comm_GetState(void);
PC_Comm_DataTypeDef* PC_Comm_GetDataPtr(void);
PC_ParsedData_t* PC_Comm_GetParsedDataPtr(void);
uint8_t PC_Comm_IsCommandOnline(uint8_t cmd_id);
uint32_t PC_Comm_GetCommandLastUpdate(uint8_t cmd_id);

#ifdef __cplusplus
}
#endif

#endif
