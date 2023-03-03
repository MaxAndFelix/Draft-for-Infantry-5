#include "bsp_usart.h"

void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{

  // disable DMA
  // 失效DMA
  __HAL_DMA_DISABLE(&hdma_usart1_tx);
  while (hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_usart1_tx);
  }

  // clear flag
  // 清除标志位
  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);
  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_HTIF7);

  // set data address
  // 设置数据地址
  hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
  // set data length
  // 设置数据长度
  hdma_usart1_tx.Instance->NDTR = len;
  // 设置目标地址
  hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);

  // enable DMA
  // 使能DMA
  __HAL_DMA_ENABLE(&hdma_usart1_tx);
}

void UART_func_init(void)
{
  // initialize();
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // 使能空闲中断
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); // 使能空闲中断

  HAL_UART_Receive_DMA(&huart1, rx_buffer1, BUFFER_SIZE);
  HAL_UART_Receive_DMA(&huart6, rx_buffer2, BUFFER_SIZE);
}
void UART_func_send(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t len)
{
  if (huart == NULL || buf == NULL)
    return;
  HAL_UART_Transmit_DMA(huart, buf, len);
}

uint8_t *UART_func_read(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
    return rx_buffer1;
  else if (huart == &huart6)
    return rx_buffer2;

  return NULL;
}

void UART_func_clear(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    if (rxFlag1)
    {
      rxLen1 = 0;
      rxFlag1 = 0;
      memset(rx_buffer1, 0, sizeof(rx_buffer1));
      HAL_UART_Receive_DMA(&huart1, (uint8_t *)rx_buffer1, sizeof(rx_buffer1));
    }
  }
  else if (huart == &huart6)
  {
    if (rxFlag2)
    {
      rxLen2 = 0;
      rxFlag2 = 0;
      memset(rx_buffer2, 0, sizeof(rx_buffer2));
      HAL_UART_Receive_DMA(&huart6, (uint8_t *)rx_buffer2, sizeof(rx_buffer2));
    }
  }
}
void UART_func_printf(const char *fmt, ...)
{
  static uint8_t tx_buf[256] = {0};
  static va_list ap;
  static uint16_t len;
  va_start(ap, fmt);
  len = vsprintf((char *)tx_buf, fmt, ap);
  va_end(ap);
  HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
  // usart1_tx_dma_enable(tx_buf, len);
}

void UART_func_printfloat(float value)
{
  int tmp, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6;
  tmp = (int)value;
  tmp1 = (int)((value - tmp) * 10) % 10;
  tmp2 = (int)((value - tmp) * 100) % 10;
  tmp3 = (int)((value - tmp) * 1000) % 10;
  tmp4 = (int)((value - tmp) * 10000) % 10;
  tmp5 = (int)((value - tmp) * 100000) % 10;
  tmp6 = (int)((value - tmp) * 1000000) % 10;
  UART_func_printf("%d.%d%d%d%d%d%d", tmp, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6);
}


void UART_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t temp;
  HAL_UART_IRQHandler(huart);
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    HAL_UART_DMAStop(huart);

    if (huart == &huart6)
    {
      temp = __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
      rxLen2 = BUFFER_SIZE - temp;
      rxFlag2 = 1;
    }
    else if (huart == &huart1)
    {
      temp = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
      rxLen1 = BUFFER_SIZE - temp;
      rxFlag1 = 1;
    }
  }
}
