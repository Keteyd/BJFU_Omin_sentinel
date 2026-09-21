#include "sys_dwt.h"

DWT_TimeTypeDef SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_us;
static uint32_t last_cycle;
uint64_t CYCCNT64;

/* The 1 ms INS/control tasks call this more often than a CYCCNT wrap (~25 s).
 * Accumulate unsigned deltas atomically: handles 2^32 rollover and IRQ reentry. */
static uint64_t DWT_ReadCycles(void) {
    uint32_t mask = __get_PRIMASK(), now;
    uint64_t result;
    __disable_irq();
    now = DWT->CYCCNT;
    CYCCNT64 += (uint32_t)(now - last_cycle);
    last_cycle = now;
    result = CYCCNT64;
    __set_PRIMASK(mask);
    return result;
}
void DWT_Init(uint32_t CPU_Freq_mHz) {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000U;
    CPU_FREQ_Hz_us = CPU_Freq_mHz;
    last_cycle = 0U; CYCCNT64 = 0U;
    SysTime.s = 0U; SysTime.ms = SysTime.us = 0U;
    __set_PRIMASK(mask);
}
float DWT_GetDeltaT(uint32_t *cnt_last) {
    uint32_t now = DWT->CYCCNT;
    float dt = (float)(uint32_t)(now - *cnt_last) / (float)CPU_FREQ_Hz;
    *cnt_last = now; (void)DWT_ReadCycles(); return dt;
}
double DWT_GetDeltaT64(uint32_t *cnt_last) {
    uint32_t now = DWT->CYCCNT;
    double dt = (double)(uint32_t)(now - *cnt_last) / (double)CPU_FREQ_Hz;
    *cnt_last = now; (void)DWT_ReadCycles(); return dt;
}
uint64_t DWT_GetTimeline_us(void) {
    return CPU_FREQ_Hz_us ? DWT_ReadCycles() / CPU_FREQ_Hz_us : 0U;
}
void DWT_SysTimeUpdate(void) {
    uint32_t mask = __get_PRIMASK(); uint64_t us;
    __disable_irq(); us = DWT_GetTimeline_us();
    SysTime.s = (uint32_t)(us / 1000000ULL);
    SysTime.ms = (uint16_t)((us / 1000ULL) % 1000ULL);
    SysTime.us = (uint16_t)(us % 1000ULL);
    __set_PRIMASK(mask);
}
float DWT_GetTimeline_s(void) { return (float)((double)DWT_GetTimeline_us() / 1000000.0); }
float DWT_GetTimeline_ms(void) { return (float)((double)DWT_GetTimeline_us() / 1000.0); }
void DWT_Delay(float delay) {
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < delay * (float)CPU_FREQ_Hz) {}
    (void)DWT_ReadCycles();
}
