#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"
#include "usart.h"
#include "memory.h"
#include "stdio.h"
#include "stdarg.h"
#define BUFFER_SIZE 100
/*
    uart3 用于接收遥控器DR16数据
    uart1 用于与上位机进行数据交互
    uart6 用作调试用串口，与PC进行数据交互
*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
uint8_t rx_buffer1[100]  ,rx_buffer2[100];
uint8_t rxFlag1, rxFlag2, rxLen1, rxLen2;
void UART_func_init(void);
void UART_func_send(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t len); // 发送数据
void UART_func_printf(const char *fmt, ...);                               // 向串口输出端printf
void UART_func_printfloat(float value);
uint8_t *UART_func_read(UART_HandleTypeDef *huart);                        // 返回数据源
void UART_func_clear(UART_HandleTypeDef *huart);                           // 清空缓存数据
void UART_IRQHandler(UART_HandleTypeDef *huart);
/*
示例：
const uint8_t *data = UART_func_read(&huart6);
其他操作...需尽快将数据拷贝（例如遥控器数据接收把sbuf转移至RC_data_ctrl）
UART_func_clear(&huart6);
*/



#endif
