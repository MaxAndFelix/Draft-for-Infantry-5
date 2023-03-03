#ifndef INS_Task_H
#define INS_Task_H
#include "struct_typedef.h"
#include "main.h"
#include "spi.h"
#include "cmsis_os.h"

#include "bsp_bmi088.h"
#include "bsp_ist8310.h"
#include "bsp_spi.h"

#include "pid.h"
#include "MahonyAHRS.h"
#include "math.h"

#define SPI_DMA_GYRO_LENGHT 8
#define SPI_DMA_ACCEL_LENGHT 9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

#define IMU_DR_SHFITS 0
#define IMU_SPI_SHFITS 1
#define IMU_UPDATE_SHFITS 2
#define IMU_NOTIFY_SHFITS 3

#define BMI088_GYRO_RX_BUF_DATA_OFFSET 1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

#define TEMPERATURE_PID_KP 1600.0f       // 温度控制PID的kp
#define TEMPERATURE_PID_KI 0.2f          // 温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f          // 温度控制PID的kd
#define TEMPERATURE_PID_MAX_OUT 4500.0f  // 温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f // 温度控制PID的max_iout

// ist83100原始数据在缓冲区buf的位置
#define IST8310_RX_BUF_DATA_OFFSET 16

#define MPU6500_TEMP_PWM_MAX 5000 // mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1

#define INS_TASK_INIT_TIME 7 // 任务开始初期 delay 一段时间

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

extern SPI_HandleTypeDef hspi1;
extern BMI088_Real_Data_T bmi088_real_data;
extern IST8310_Real_Data_T ist8310_real_data;
extern fp32 INS_angle[3];

extern TIM_HandleTypeDef htim10;
extern void Info_Proc(void const *pvParameters); // imu任务, 初始化 bmi088, ist8310, 计算欧拉角

// extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);
// extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3]);
#endif