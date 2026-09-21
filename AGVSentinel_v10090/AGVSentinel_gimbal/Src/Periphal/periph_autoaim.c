#include "periph_autoaim.h"
#include "module_uart_mux.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx; 
extern DMA_HandleTypeDef hdma_usart1_tx;  

static uint16_t s_rx_cursor;

void AutoAim_RxDmaPoll(void)
{
    uint16_t remaining = (uint16_t)hdma_usart1_rx.Instance->NDTR;
    if (remaining > ChariotRecognition_data_dma_buf_len) return;
    __DMB();
    UartMux_Drain(&s_rx_cursor, ChariqotRecognition_data[0],
        ChariotRecognition_data_dma_buf_len,
        (uint16_t)(ChariotRecognition_data_dma_buf_len - remaining), AutoAim_ProcessUartData);
}

static void AutoAim_RxDmaComplete(DMA_HandleTypeDef *dma)
{
    if (dma == &hdma_usart1_rx) AutoAim_RxDmaPoll();
}



//UART1的tx的DMA模式初始化
void usart1_tx_dma_init(void)
{
    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }
      
      hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
		
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;
}
//UART1的rx的DMA模式初始化
void usart1_rx_dma_init(void)
{
    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
		 __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);   //串口1空闲中断使能，少了这句不能进串口1的中断服务函数

	  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_HISR_TCIF5);
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(&ChariqotRecognition_data[0][0]);   //自瞄接收数据数组地址1   双缓冲区
		hdma_usart1_rx.Instance->M1AR = (uint32_t)(&ChariqotRecognition_data[1][0]);		//自瞄接收数据数组地址2
		
    hdma_usart1_rx.Instance->NDTR = (uint16_t)ChariotRecognition_data_dma_buf_len;  //接收数据长度
    /* Circular RX stays running across IDLE. HT/TC prevent full-bank loss. */
    CLEAR_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM | DMA_SxCR_CT);
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_CIRC);
    s_rx_cursor = 0U;
    hdma_usart1_rx.XferHalfCpltCallback = AutoAim_RxDmaComplete;
    hdma_usart1_rx.XferCpltCallback = AutoAim_RxDmaComplete;
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_HISR_HTIF5 | DMA_HISR_TCIF5);
    __HAL_DMA_ENABLE_IT(&hdma_usart1_rx, DMA_IT_HT | DMA_IT_TC);
	  __HAL_DMA_ENABLE(&hdma_usart1_rx);		
}

void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}

void usart1_rx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}
