#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "can.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

// rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

uint8_t txData_can1[8];
uint8_t txData_can2[8];

uint8_t rxData_can1[8];
uint8_t rxData_can2[8];
void CAN_func_init(void);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan); // Can 发送中断回调  此中断会在Can数据发送成功后调用
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);  // Can 接收中断回调  此中断会在Can通道将要接收数据使用，因此需要使用HAL_CAN_GetRxMessage函数接收数据
CAN_TxHeaderTypeDef txHeader_can1;
CAN_TxHeaderTypeDef txHeader_can2;

CAN_RxHeaderTypeDef rxHeader_can1;
CAN_RxHeaderTypeDef rxHeader_can2;
#endif
