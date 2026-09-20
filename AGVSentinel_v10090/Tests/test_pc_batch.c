#include <assert.h>
#include <stdio.h>
#include "../AGVSentinel_gimbal/Src/Periphal/periph_pc_comm.c"

UART_HandleTypeDef huart1;
static uint32_t mask;
static uint8_t result, *dma_source;
static uint16_t dma_size;
static Remote_RemoteDataTypeDef remote;
uint32_t HAL_GetTick(void) { return 0U; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1U; }
void __set_PRIMASK(uint32_t value) { mask = value; }
Remote_RemoteDataTypeDef *Remote_GetRemoteDataPtr(void) { return &remote; }
uint16_t Uart_DMACurrentDataCounter(void *instance) { (void)instance; return 0U; }
void YawIdentApp_Receive(const uint8_t data[12]) { (void)data; }
uint8_t YawIdentApp_OwnsControl(void) { return 0U; }
uint8_t HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size) {
    assert(mask == 1U && uart == &huart1 && uart->gState == HAL_UART_STATE_READY);
    if (!result) { dma_source = data; dma_size = size; uart->gState = 1U; }
    return result;
}
uint8_t HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size) {
    assert(uart == &huart1 && uart->gState == HAL_UART_STATE_READY);
    assert(size == 16U && PC_Comm_VerifyChecksum(data));
    uart->gState = 1U;
    return HAL_OK;
}

int main(void) {
    uint8_t payload[72], saved[96];
    uint8_t trace[CAN_TRACE_BYTES], trace_saved[CAN_TRACE_FRAME_BYTES];
    unsigned count, i, prior;
    assert(PC_Comm_CalculateCRC8((const uint8_t *)"123456789", 9U) == 0xa2U);
    for (i = 0U; i < sizeof(payload); ++i) payload[i] = (uint8_t)(i * 3U);
    for (count = 1U; count <= 6U; ++count) {
        huart1.gState = HAL_UART_STATE_READY;
        assert(PC_Comm_SendBatch(0x3dU, payload, (uint8_t)count));
        assert(mask == 0U && dma_size == count * 16U && dma_source != payload);
        memcpy(saved, dma_source, dma_size);
        for (i = 0U; i < count; ++i) {
            const uint8_t *frame = dma_source + i * 16U;
            assert(frame[0] == 0xffU && frame[1] == 0x3dU && frame[15] == 0x0dU);
            assert(!memcmp(frame + 2U, payload + i * 12U, 12U));
            assert(PC_Comm_VerifyChecksum(frame));
        }
        assert(!PC_Comm_SendBatch(0xffU, payload, 6U));
        assert(!PC_Comm_SendPacket(0x20U, payload, 12U));
        assert(!memcmp(saved, dma_source, dma_size) && mask == 0U);
    }
    for (prior = 0U; prior < 2U; ++prior) {
        mask = prior; huart1.gState = HAL_UART_STATE_READY; result = 1U;
        assert(!PC_Comm_SendBatch(0x3dU, payload, 6U) && mask == prior);
        result = 0U;
        assert(PC_Comm_SendBatch(0x3dU, payload, 6U) && mask == prior);
        assert(!PC_Comm_SendBatch(0x3dU, payload, 6U) && mask == prior);
    }
    mask = 0U; huart1.gState = HAL_UART_STATE_READY;
    assert(!PC_Comm_SendBatch(0x3dU, NULL, 1U));
    assert(!PC_Comm_SendBatch(0x3dU, payload, 0U));
    assert(!PC_Comm_SendBatch(0x3dU, payload, 7U));
    Const_PC_Comm_UART_HANDLER = NULL;
    assert(!PC_Comm_SendBatch(0x3dU, payload, 1U));
    Const_PC_Comm_UART_HANDLER = &huart1;
    assert(PC_Comm_SendPacket(0x20U, payload, 12U));
    for (i = 0U; i < sizeof(trace); ++i) trace[i] = (uint8_t)i;
    huart1.gState = HAL_UART_STATE_READY;
    assert(PC_Comm_SendTrace(0x12345678U, 5001U, trace));
    assert(dma_size == CAN_TRACE_FRAME_BYTES && dma_source != trace && mask == 0U);
    memcpy(trace_saved, dma_source, sizeof(trace_saved));
    assert(dma_source[0] == 0xffU && dma_source[1] == 0x3fU && dma_source[162] == 0x0dU);
    assert(!memcmp(dma_source + 2U, "\x78\x56\x34\x12\x89\x13", 6U));
    assert(!memcmp(dma_source + 8U, trace, sizeof(trace)));
    assert(CanTrace_Crc16(dma_source, 160U) == (uint16_t)(dma_source[160] | dma_source[161] << 8U));
    memset(trace, 0xa5, sizeof(trace)); /* Caller may reuse its ring slot after receipt. */
    assert(!PC_Comm_SendTrace(1U, 2U, trace));
    assert(!PC_Comm_SendBatch(0x3dU, payload, 6U));
    assert(!PC_Comm_SendPacket(0x20U, payload, 12U));
    assert(!memcmp(trace_saved, dma_source, sizeof(trace_saved)));
    for (prior = 0U; prior < 2U; ++prior) {
        mask = prior; huart1.gState = HAL_UART_STATE_READY; result = 1U;
        assert(!PC_Comm_SendTrace(1U, 1U, trace) && mask == prior);
        result = 0U;
        assert(PC_Comm_SendTrace(1U, 1U, trace) && mask == prior);
    }
    mask = 0U; huart1.gState = HAL_UART_STATE_READY;
    assert(!PC_Comm_SendTrace(1U, 1U, NULL));
    Const_PC_Comm_UART_HANDLER = NULL;
    assert(!PC_Comm_SendTrace(1U, 1U, trace));
    puts("Production PC DMA batching/trace: CRC, bounds, busy lifetime and IRQ restoration passed.");
    return 0;
}
