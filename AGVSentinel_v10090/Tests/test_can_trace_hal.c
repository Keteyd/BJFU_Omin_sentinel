#include <assert.h>
#include <stdio.h>
#include "../AGVSentinel_gimbal/Src/Utility/util_can.c"
Dwt fake_dwt; Debug fake_debug;
static Registers reg1, reg2;
CAN_HandleTypeDef hcan1 = {&reg1}, hcan2 = {&reg2};
static uint32_t mask_value, free_slots = 3, add_status, chosen_slot = 1, decoded;
static uint32_t free_queries;
uint32_t __get_PRIMASK(void) { return mask_value; }
void __disable_irq(void) { mask_value = 1; }
void __set_PRIMASK(uint32_t mask) { mask_value = mask; }
void fake_clear(CAN_HandleTypeDef *bus, uint32_t flag) { bus->Instance->TSR &= ~(15U << (flag & 255U)); }
uint32_t HAL_CAN_ConfigFilter(CAN_HandleTypeDef *b, CAN_FilterTypeDef *c) { (void)b; (void)c; return 0; }
uint32_t HAL_CAN_Start(CAN_HandleTypeDef *b) { (void)b; return 0; }
uint32_t HAL_CAN_ActivateNotification(CAN_HandleTypeDef *b, uint32_t v) { (void)b; assert(v == CAN_IT_RX_FIFO0_MSG_PENDING); return 0; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *b) { (void)b; ++free_queries; fake_dwt.CYCCNT += 168; return free_slots; }
uint32_t HAL_CAN_AddTxMessage(CAN_HandleTypeDef *b, CAN_TxHeaderTypeDef *h, uint8_t *d, uint32_t *m) {
    (void)b; (void)h; (void)d; if (!add_status) *m = chosen_slot; return add_status;
}
uint32_t HAL_CAN_GetRxMessage(CAN_HandleTypeDef *b, uint32_t f, CAN_RxHeaderTypeDef *h, uint8_t *d) {
    (void)b; (void)f; (void)h; (void)d; return 0;
}
void Comm_BoardLinkRxCanFrame(uint32_t i, uint8_t *d, uint32_t n) { (void)i; (void)d; (void)n; }
void RobotActuators_DecodeCan(CAN_HandleTypeDef *b, uint32_t i, uint8_t *d, uint32_t n) { (void)b; (void)i; (void)d; (void)n; ++decoded; }
int main(void) {
    CAN_TxHeaderTypeDef header;
    CanTrace_AxisRecord out[2]; uint32_t now;
    uint8_t bytes[8] = {3, 232, 0xff, 0xfe, 0xfe, 0xd4, 0, 0};
    Can_InitFilterAndStart(&hcan1); Can_TraceBegin();
    Can_InitTxHeader(&header, 0x2ff, 0, 8);
    Can_SendMessage(&hcan1, &header, bytes);
    assert(s_trace.pending[0].sequence == 1 && s_trace.axes[0].r.command == 1000);
    fake_dwt.CYCCNT = 168000; reg1.TSR = 3;
    /* Non-yaw reuse of the same slot must first finish the original yaw ID. */
    header.StdId = 0x200; Can_SendMessage(&hcan1, &header, bytes);
    reg1.TSR = 9; Can_TraceSnapshot(&now, out);
    assert(out[0].completed == 1 && out[0].complete_sequence == 1 && out[0].errors == 0);
    assert(out[0].complete_us == 1001 && mask_value == 0);
    header.StdId = 0x2ff; Can_SendMessage(&hcan1, &header, bytes);
    reg1.TSR = 3; header.StdId = 1; /* Production DM/Pitch uses this immediate path. */
    {
        uint32_t before = free_queries;
        assert(Can_SendMessageNoWait(&hcan1, &header, bytes) == HAL_OK);
        assert(free_queries == before); /* No new mailbox wait for Pitch. */
    }
    reg1.TSR = 9; Can_TraceSnapshot(&now, out);
    assert(out[0].completed == 1 && out[0].errors == 0 && out[0].complete_sequence == 2);
    header.StdId = 0x1ff; chosen_slot = 2;
    Can_SendMessage(&hcan2, &header, bytes);
    assert(s_trace.axes[1].r.command == -2 && s_trace.pending[4].axis == 2);
    reg2.TSR = 9U << 8; Can_TraceSnapshot(&now, out);
    assert(out[1].errors == 1 && !(out[1].flags >> 8));
    header.StdId = 0x206; Can_RxMessageCallback(&hcan2, &header, bytes);
    assert(decoded == 1 && s_trace.axes[1].r.current == -300);
    header.StdId = 0x2ff; free_slots = 0; add_status = 1;
    Can_SendMessage(&hcan1, &header, bytes); Can_TraceSnapshot(&now, out);
    assert(out[0].failed == 1 && (out[0].flags & CT_WAIT_EXHAUSTED) && out[0].wait_us >= 5000);
    Can_TraceEnd(); free_slots = 3; add_status = 0;
    Can_SendMessage(&hcan1, &header, bytes); assert(!s_trace.active);
    puts("Production CAN path: reuse, enqueue failure, TSR errors, current and timestamp passed.");
    return 0;
}
