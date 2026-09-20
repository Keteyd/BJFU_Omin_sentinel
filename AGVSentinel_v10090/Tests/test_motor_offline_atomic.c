#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
typedef struct { uint32_t last_update_time; uint8_t init, is_online, type; } Motor_MotorTypeDef;
enum { Motor_TYPE_NOT_CONNECTED = 0, Const_Motor_MOTOR_OFFLINE_TIME = 100 };
static uint32_t mask, tick;
static int inject;
static Motor_MotorTypeDef *target;
static uint32_t __get_PRIMASK(void) { return mask; }
static void __disable_irq(void) { mask = 1U; }
static void __set_PRIMASK(uint32_t value) { mask = value; }
static uint32_t HAL_GetTick(void) {
    uint32_t result = tick;
    if (inject && !mask) {
        target->last_update_time = ++tick;
        target->is_online = 1U;
    }
    return result;
}
#include "motor_offline_under_test.inc"
static uint8_t vulnerable_fresh(Motor_MotorTypeDef *m, uint32_t timeout) {
    uint32_t now = HAL_GetTick();
    return (uint8_t)(m->init && now - m->last_update_time <= timeout);
}
int main(void) {
    Motor_MotorTypeDef m = {100U, 1U, 1U, 1U};
    target = &m; tick = 100U; inject = 1;
    assert(!Motor_IsMotorOffline(&m) && m.is_online && !mask);
    inject = 0; tick = 201U;
    assert(Motor_IsMotorOffline(&m) && !m.is_online && !mask);
    tick = 2U; m.last_update_time = 0xfffffffeU;
    mask = 1U;
    assert(!Motor_IsMotorOffline(&m) && m.is_online && mask == 1U);
    m.init = 0U;
    assert(Motor_IsMotorOffline(&m));
    assert(!Motor_IsMotorOffline(NULL));
    m = (Motor_MotorTypeDef){100U, 1U, 1U, 1U};
    target = &m; tick = 100U; mask = 0U; inject = 1;
    assert(!vulnerable_fresh(&m, 50U));
    m.last_update_time = tick = 100U;
    assert(Motor_IsMotorFeedbackFresh(&m, 50U) && !mask);
    inject = 0; tick = 151U;
    assert(!Motor_IsMotorFeedbackFresh(&m, 50U));
    tick = 2U; m.last_update_time = 0xfffffffeU; mask = 1U;
    assert(Motor_IsMotorFeedbackFresh(&m, 50U) && mask == 1U);
    m.init = 0U;
    assert(!Motor_IsMotorFeedbackFresh(&m, 50U));
    assert(!Motor_IsMotorFeedbackFresh(NULL, 50U));
    m.type = Motor_TYPE_NOT_CONNECTED;
    assert(Motor_IsMotorFeedbackFresh(&m, 50U));
    puts("Production motor feedback decisions: atomic timestamp, timeout, wrap and IRQ preservation passed.");
    return 0;
}
