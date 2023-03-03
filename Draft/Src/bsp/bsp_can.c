#include "bsp_can.h"

void CAN_func_init()
{

  // 本函数用于配置can必需的配置：过滤器、使能、开启回调
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x000;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x000;
  canfilterconfig.FilterMaskIdLow = 0x000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.FilterBank = 0;            // which filter bank to use from the assigned ones
  canfilterconfig.SlaveStartFilterBank = 14; // how many filters to assign to the CAN1 (master can)

  if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK)
    Error_Handler();
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
    Error_Handler();
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    Error_Handler();

  canfilterconfig.FilterBank = 14;            // which filter bank to use from the assigned ones
  canfilterconfig.SlaveStartFilterBank = 14; // how many filters to assign to the CAN1 (master can)

  if (HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig) != HAL_OK)
    Error_Handler();
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
    Error_Handler();
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    Error_Handler();
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{


    /*
        Can 发送中断回调 
        此中断会在Can数据发送成功后调用
    */

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /*
        Can 接收中断回调
        此中断会在Can通道将要接收数据使用，因此需要使用HAL_CAN_GetRxMessage函数接收数据
    */
    if (hcan == &hcan1)
    {
        // HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1_RxHeader, can1_RxData); //将数据传输到RxData中
        // //usart_printf("%d\n",can1_RxHeader.StdId);
        // if (can1_RxHeader.StdId == 0x300)
        // {
        //     rc_tmp[0] = (can1_RxData[0] << 8) | can1_RxData[1];
        //     rc_tmp[1] = (can1_RxData[2] << 8) | can1_RxData[3];
        //     rc_tmp[2] = (can1_RxData[4] << 8) | can1_RxData[5];
        //     rc_tmp[3] = (can1_RxData[6] << 8) | can1_RxData[7];
        // }
        // else if (can1_RxHeader.StdId == 0x301)
        // {
        //     usart_printf("ch0:%d,ch1:%d,ch2:%d,ch3:%d,ch4:%d\n", rc_tmp[0], rc_tmp[1], rc_tmp[2], rc_tmp[3], ((can1_RxData[0] << 8) | can1_RxData[1]), can1_RxData[2], can1_RxData[3]);
        // }
    }
    
    else if (hcan == &hcan2)
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rxHeader_can2, rxData_can2);

    
}
