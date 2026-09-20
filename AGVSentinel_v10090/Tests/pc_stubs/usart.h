#ifndef PC_TEST_USART_H
#define PC_TEST_USART_H
#include <stdint.h>
typedef struct { void *Instance; } DMA_HandleTypeDef;
typedef struct { void *Instance; DMA_HandleTypeDef *hdmarx; uint32_t gState; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1;
#define HAL_UART_STATE_READY 0U
#define HAL_OK 0U
#define __HAL_DMA_DISABLE(x) ((void)(x))
#define __HAL_DMA_ENABLE(x) ((void)(x))
#define __HAL_DMA_SET_COUNTER(x, n) ((void)(x), (void)(n))
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
uint8_t HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size);
uint8_t HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size);
#endif
