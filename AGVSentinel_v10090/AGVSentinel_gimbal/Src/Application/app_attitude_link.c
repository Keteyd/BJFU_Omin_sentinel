#include "app_attitude_link.h"
#include "app_ins.h"
#include "attitude_wire.h"
#include "sys_dwt.h"
#include "usart.h"
#include <string.h>

_Static_assert(sizeof(float) == 4, "wire protocol requires float32");

/* IRQ/task sharing is protected by PRIMASK. The only live DMA buffer is tx[].
 * Legacy CAN trace packets are 163 bytes, larger than new protocol's 96-byte cap. */
static uint8_t tx[192], attitude[AV_MAX_FRAME];
static uint8_t active, ack_pending, sync_pending, attitude_pending;
static uint64_t session, request_t1, request_t2;
static uint32_t last_request_ms, last_attitude_ms, request_sequence, tx_sequence;

uint8_t AttitudeLink_IsOnline(void) {
    uint32_t mask = __get_PRIMASK(); uint8_t online;
    __disable_irq();
    online = (uint8_t)(active && (uint32_t)(HAL_GetTick() - last_request_ms) < 3000U);
    __set_PRIMASK(mask); return online;
}
uint8_t AttitudeLink_Validate(const uint8_t *p, uint16_t n) {
    if (!p || n < 20U || n > AV_MAX_FRAME) return 0U;
    return (uint8_t)(p[0] == 0xa6U && p[1] == 0x5aU && p[2] == 1U &&
        AvRead(p + 4, 2) + 20U == n && AvRead(p + n - 2U, 2) == AvCrc(p, n - 2U));
}
void AttitudeLink_Receive(const uint8_t *p, uint16_t n) {
    uint32_t mask; uint64_t incoming;
    if (!AttitudeLink_Validate(p, n)) return;
    incoming = AvRead(p + 10, 8);
    if (!incoming) return;
    mask = __get_PRIMASK(); __disable_irq();
    if (p[3] == AV_HELLO && n == 20U) {
        if (!active || session != incoming) {
            tx_sequence = 0U; attitude_pending = 0U; last_attitude_ms = HAL_GetTick();
        }
        session = incoming; active = 1U; ack_pending = 1U; sync_pending = 0U;
        request_sequence = (uint32_t)AvRead(p + 6, 4); last_request_ms = HAL_GetTick();
    } else if (p[3] == AV_SYNC && n == 28U && AttitudeLink_IsOnline() && incoming == session) {
        request_sequence = (uint32_t)AvRead(p + 6, 4);
        request_t1 = AvRead(p + AV_HEADER, 8);
        request_t2 = DWT_GetTimeline_us(); /* Complete request received, not first wire bit. */
        sync_pending = 1U; last_request_ms = HAL_GetTick();
    }
    __set_PRIMASK(mask);
}

/* Caller holds PRIMASK; HAL completion owns tx until gState becomes READY. */
static uint8_t StartTx(const uint8_t *data, uint16_t len) {
    if (huart1.gState != HAL_UART_STATE_READY || len > sizeof(tx)) return 0U;
    if (data != tx) memcpy(tx, data, len);
    return (uint8_t)(HAL_UART_Transmit_DMA(&huart1, tx, len) == HAL_OK);
}
uint8_t AttitudeLink_SendLegacy(const uint8_t *data, uint16_t len) {
    uint32_t mask; uint8_t sent = 0U;
    if (!data || !len || len > sizeof(tx)) return 0U;
    mask = __get_PRIMASK(); __disable_irq();
    if (!AttitudeLink_IsOnline() || (!ack_pending && !sync_pending && !attitude_pending &&
        (uint32_t)(HAL_GetTick() - last_attitude_ms) < 5U)) sent = StartTx(data, len);
    __set_PRIMASK(mask); return sent;
}
void AttitudeLink_Service(void) {
    uint32_t mask = __get_PRIMASK(), now = HAL_GetTick(); unsigned n, i;
    __disable_irq();
    if (!AttitudeLink_IsOnline()) {
        active = 0U; ack_pending = sync_pending = attitude_pending = 0U;
        __set_PRIMASK(mask); return;
    }
    if ((uint32_t)(now - last_attitude_ms) >= 5U) {
        INS_Observation imu;
        uint8_t *p = attitude + AV_HEADER;
        last_attitude_ms = now;
        INS_ReadObservation(&imu);
        n = AvBegin(attitude, AV_ATTITUDE, 42U, ++tx_sequence, session);
        AvWrite(p, imu.sample_time_us, 8); AvWrite(p + 8, imu.sequence, 4);
        AvWrite(p + 12, INS_IsReady() ? 1U : 0U, 2);
        for (i = 0; i < 4U; ++i) AvPutFloat(p + 14U + i * 4U, imu.q[i]);
        for (i = 0; i < 3U; ++i) AvPutFloat(p + 30U + i * 4U, imu.gyro[i]);
        AvFinish(attitude, n); attitude_pending = 1U;
    }
    if (huart1.gState == HAL_UART_STATE_READY) {
        if (ack_pending) {
            n = AvBegin(tx, AV_ACK, 0U, request_sequence, session); AvFinish(tx, n);
            if (StartTx(tx, (uint16_t)n)) ack_pending = 0U;
        } else if (sync_pending) {
            n = AvBegin(tx, AV_SYNC_REPLY, 24U, request_sequence, session);
            AvWrite(tx + AV_HEADER, request_t1, 8);
            AvWrite(tx + AV_HEADER + 8U, request_t2, 8);
            AvWrite(tx + AV_HEADER + 16U, DWT_GetTimeline_us(), 8);
            AvFinish(tx, n);
            if (StartTx(tx, (uint16_t)n)) sync_pending = 0U;
        } else if (attitude_pending) {
            if (StartTx(attitude, 62U)) attitude_pending = 0U;
        }
    }
    __set_PRIMASK(mask);
}
