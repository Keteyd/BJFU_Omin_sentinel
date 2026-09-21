/*
 * Project      : Infantry_Neptune
 * File         : periph_pc_comm.h
 * Description  : Fixed-length PC frames multiplexed on the USART1 vision link.
 */

#ifndef PERIPH_PC_COMM_H
#define PERIPH_PC_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include <stdint.h>
#include "module_big_yaw_full.h"

#define PC_COMM_PACKET_LEN          16U
#define PC_COMM_HEADER_SOF          0xFFU
#define PC_COMM_TAIL_EOF            0x0DU
#define PC_COMM_DATA_LEN            13U
#define PC_COMM_CMD_PAYLOAD_LEN     (PC_COMM_DATA_LEN - 1U)
#define PC_COMM_RX_BUFF_LEN         64U
#define PC_COMM_CRC_OFFSET          14U
#define PC_COMM_TAIL_OFFSET         15U
#define PC_COMM_OFFLINE_TIME        100U
#define PC_COMM_TUNE_SESSION_TIME   750U

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
    uint32_t gimbal_tune_sequence;
    uint32_t gimbal_tune_action_sequence;
    uint32_t pitch_tune_sequence;
    uint32_t pitch_tune_action_sequence;
    uint32_t big_yaw_tune_sequence;
    uint32_t big_yaw_tune_rx_time;
    uint8_t big_yaw_tune_rx_safe;
    uint8_t big_yaw_tune_full;
    uint8_t big_yaw_full_status;
    uint16_t big_yaw_full_id;
    uint32_t gimbal_tune_session_time;
    uint8_t gimbal_tune_session_seen;
    uint8_t active_tune_axis;
    uint8_t tune_monitor_only;
    uint8_t yaw_capture_requested;
} PC_Comm_DataTypeDef;

typedef enum {
    PC_CMD_GIMBAL          = 0x01,
    PC_CMD_CHASSIS         = 0x02,
    PC_CMD_NAVIGATION      = 0x03,
    PC_CMD_POWER_HEAT      = 0x10,
    PC_CMD_DAMAGE_HEALTH   = 0x11,
    PC_CMD_SHOOT_STATUS    = 0x12,
    PC_CMD_FIELD_BUFF      = 0x13,
    PC_CMD_GIMBAL_PID_TUNE = 0x20,
    PC_CMD_GIMBAL_PID_DATA = 0x21,
    PC_CMD_GIMBAL_PID_AUX  = 0x22,
    PC_CMD_GIMBAL_PID_ACTION = 0x23,
    PC_CMD_GIMBAL_PID_SESSION = 0x24,
    PC_CMD_PITCH_PID_TUNE   = 0x25,
    PC_CMD_PITCH_PID_DATA   = 0x26,
    PC_CMD_PITCH_PID_AUX    = 0x27,
    PC_CMD_PITCH_PID_ACTION = 0x28,
    PC_CMD_PITCH_PID_LIMITS = 0x29,
    PC_CMD_YAW_ENCODERS    = 0x2A,
    PC_CMD_YAW_LIMITS      = 0x2B,
    PC_CMD_YAW_COORDINATOR = 0x2C,
    PC_CMD_BIG_YAW_TUNE    = 0x2D,
    PC_CMD_BIG_YAW_CONFIG  = 0x2E,
    PC_CMD_BIG_YAW_PEAKS   = 0x2F,
    PC_CMD_BIG_YAW_FULL_TUNE = 0x30,
    PC_CMD_BIG_YAW_FULL_CONFIG = 0x31,
    PC_CMD_YAW_CAPTURE = 0x32,
    PC_CMD_IMU_READ_DIAG = 0x34,
    PC_CMD_IMU_TIMING_DIAG = 0x35
} PC_Comm_CmdID_e;

#define PC_GIMBAL_MODE_ENABLE  (1U << 0)
#define PC_GIMBAL_MODE_AUTOAIM (1U << 1)
#define PC_GIMBAL_PID_ACTION_RESET (1U << 0)
#define PC_GIMBAL_PID_ACTION_STEP  (1U << 1)
#define PC_PITCH_PID_TUNE_ENABLE   (1U << 0)
#define PC_PITCH_PID_ACTION_RESET  (1U << 0)
#define PC_PITCH_PID_ACTION_STEP   (1U << 1)
#define PC_PITCH_PID_ACTION_MIT_GAINS (1U << 2)
#define PC_TUNE_AXIS_SMALL_YAW     1U
#define PC_TUNE_AXIS_PITCH         2U
#define PC_TUNE_SESSION_MONITOR   0xA5U

#pragma pack(push, 1)

typedef struct {
    uint16_t angle_kp_milli;
    uint16_t speed_kp_milli;
    uint16_t effort_milli;
    uint16_t filter_tau_1e4_s;
    uint16_t request_id;
    uint8_t magic; /* 0xB7 */
    uint8_t version; /* 1 */
} PC_Recv_BigYawTune_t;

typedef struct {
    uint16_t angle_kp_milli;
    uint16_t speed_kp_milli;
    uint16_t effort_milli;
    uint16_t filter_tau_1e4_s;
    uint16_t request_id;
    uint8_t status;
    uint8_t flags; /* 0 SAFE, 1 outputs off, 2 active, 3 saturated, 4 online, 7 V1. */
} PC_Send_BigYawConfig_t;

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
    uint16_t angle_kp_milli;
    uint16_t speed_kp_milli;
    uint16_t speed_limit_centi_rpm;
    uint16_t speed_filter_alpha_1e4;
    uint16_t manual_step_1e4_deg;
    uint16_t effort_limit_milli;
} PC_Recv_GimbalPidTune_t;

typedef struct {
    int16_t step_centi_deg;
    uint8_t flags;
    uint8_t reserved[9];
} PC_Recv_GimbalPidAction_t;

typedef struct {
    uint16_t ref_centi_deg;
    uint16_t fdb_centi_deg;
    int16_t error_centi_deg;
    int16_t speed_ref_deci_rpm;
    int16_t speed_deci_rpm;
    int16_t effort_command;
} PC_Send_GimbalPidData_t;

typedef struct {
    int16_t raw_speed_deci_rpm;
    int16_t current;
    uint8_t motor_online;
    uint8_t axis_enabled;
    uint8_t reserved[6];
} PC_Send_GimbalPidAux_t;

typedef struct {
    uint16_t imu_kp_milli;
    uint16_t imu_kd_milli;
    uint16_t rate_limit_milli_rad_s;
    int16_t gravity_effort_milli;
    uint16_t angle_deadband_1e5_rad;
    uint8_t rate_deadband_milli_rad_s;
    uint8_t flags;
} PC_Recv_PitchPidTune_t;

typedef struct {
    int16_t step_milli_rad;
    uint8_t flags;
    uint16_t mit_kp_centi;
    uint16_t mit_kd_milli;
    uint8_t reserved[5];
} PC_Recv_PitchPidAction_t;

typedef struct {
    int16_t ref_milli_rad;
    int16_t fdb_milli_rad;
    int16_t error_milli_rad;
    int16_t motor_ref_milli_rad;
    int16_t motor_fdb_milli_rad;
    int16_t motor_rate_ref_milli_rad_s;
} PC_Send_PitchPidData_t;

typedef struct {
    int16_t imu_rate_milli_rad_s;
    int16_t gravity_effort_milli;
    int16_t motor_speed_milli_rad_s;
    int16_t motor_effort_milli;
    uint8_t motor_online;
    uint8_t axis_enabled;
    int16_t applied_step_milli_rad;
} PC_Send_PitchPidAux_t;

typedef struct {
    int16_t motor_min_mrad;
    int16_t motor_max_mrad;
    int16_t imu_min_mrad;
    int16_t imu_max_mrad;
    uint8_t limits_initialized;
    uint8_t dm_state;
    uint8_t monitor_only;
    uint8_t protocol_version;
} PC_Send_PitchPidLimits_t;

typedef struct {
    uint16_t small_raw_count;
    uint16_t big_raw_count;
    uint16_t small_age_ms;
    uint16_t big_age_ms;
    uint8_t feedback_valid_mask; /* Bit 0: small; bit 1: big. */
    uint8_t yaw_output_active;
    uint8_t monitor_only;
    uint8_t protocol_version;
} PC_Send_YawEncoders_t;

typedef struct {
    int16_t joint_centi_deg;
    int16_t minimum_centi_deg;
    int16_t maximum_centi_deg;
    int16_t speed_ref_deci_rpm;
    uint8_t limits_valid;
    uint8_t limit_status;
    uint8_t big_yaw_enabled;
    uint8_t protocol_version;
} PC_Send_YawLimits_t;

typedef struct {
    int16_t big_error_centi_deg;
    int16_t big_speed_ref_centi_rpm;
    int16_t big_speed_fdb_centi_rpm;
    int16_t big_effort_milli;
    uint8_t mode;
    uint8_t active;
    uint8_t relief_negative;
    uint8_t protocol_version;
} PC_Send_YawCoordinator_t;

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
extern PC_Recv_GimbalPidTune_t PC_GimbalPidTune;
extern PC_Recv_GimbalPidAction_t PC_GimbalPidAction;
extern PC_Recv_PitchPidTune_t PC_PitchPidTune;
extern PC_Recv_PitchPidAction_t PC_PitchPidAction;
extern PC_Recv_BigYawTune_t PC_BigYawTune;
extern float PC_BigYawFullValues[BIG_YAW_FULL_COUNT];
uint8_t PC_Comm_IsRemoteSafe(void);

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
uint32_t PC_Comm_GetGimbalTuneSequence(void);
uint32_t PC_Comm_GetGimbalTuneActionSequence(void);
uint32_t PC_Comm_GetPitchTuneSequence(void);
uint32_t PC_Comm_GetPitchTuneActionSequence(void);
uint8_t PC_Comm_GetActiveTuneAxis(void);
uint8_t PC_Comm_IsGimbalTuneSessionOnline(void);
uint8_t PC_Comm_IsYawCaptureOnline(void);
uint8_t PC_Comm_IsGimbalTuneControlLocked(void);
const PC_Recv_GimbalPidTune_t* PC_Comm_GetGimbalTune(void);
const PC_Recv_GimbalPidAction_t* PC_Comm_GetGimbalTuneAction(void);
const PC_Recv_PitchPidTune_t* PC_Comm_GetPitchTune(void);
const PC_Recv_PitchPidAction_t* PC_Comm_GetPitchTuneAction(void);
uint8_t PC_Comm_SendPacket(uint8_t cmd_id, const void *payload, uint16_t payload_len);
/* Up to six 12-byte payloads, copied to DMA-safe storage before returning. */
uint8_t PC_Comm_SendBatch(uint8_t cmd_id, const uint8_t *payloads, uint8_t count);
uint8_t PC_Comm_SendTrace(uint32_t trial_id, uint16_t sequence, const void *record);

#ifdef __cplusplus
}
#endif

#endif
