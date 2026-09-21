#ifndef APP_ATTITUDE_LINK_H
#define APP_ATTITUDE_LINK_H
#include <stdint.h>
/* USART1 sole TX owner. No control/motor commands are accepted by this protocol. */
uint8_t AttitudeLink_IsOnline(void);
uint8_t AttitudeLink_Validate(const uint8_t *frame, uint16_t len);
/* Called by RX ISR: copies a bounded request only; processing/TX happens in task. */
void AttitudeLink_Receive(const uint8_t *frame, uint16_t len);
/* Called from gimbal task every ~1 ms. Latest attitude replaces unsent old data. */
void AttitudeLink_Service(void);
/* Copies legacy bytes to owned DMA buffer; returns 0 if busy or higher priority pending. */
uint8_t AttitudeLink_SendLegacy(const uint8_t *data, uint16_t len);
#endif
