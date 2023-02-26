/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             ��Ҫ����������bmi088��������ist8310��������?���㣬�ó�ŷ���ǣ�
  *             �ṩͨ��bmi088��data ready �ж�����ⲿ�������������ݵȴ��ӳ�?
  *             ͨ��DMA��SPI�����ԼCPUʱ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef INS_Task_H
#define INS_Task_H
#include "struct_typedef.h"


#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4


#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2
#define IMU_NOTIFY_SHFITS    3


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

//ist83100ԭʼ�����ڻ�����buf��λ��
#define IST8310_RX_BUF_DATA_OFFSET 16


#define TEMPERATURE_PID_KP 1600.0f //�¶ȿ���PID��kp
#define TEMPERATURE_PID_KI 0.2f    //�¶ȿ���PID��ki
#define TEMPERATURE_PID_KD 0.0f    //�¶ȿ���PID��kd

#define TEMPERATURE_PID_MAX_OUT   4500.0f //�¶ȿ���PID��max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  //�¶ȿ���PID��max_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500�����¶ȵ�����TIM������ֵ������PWM����? MPU6500_TEMP_PWM_MAX - 1


#define INS_TASK_INIT_TIME 7 //����ʼ���� delay һ��ʱ��

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu����, ��ʼ�� bmi088, ist8310, ����ŷ����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void INS_Task(void const *pvParameters);
extern void usart_printf(const char *fmt,...);
/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
/**
  * @brief          У׼������
  * @param[out]     �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[out]     �����ǵ���Ư���ɼ������ǵľ�ֹ�������Ϊoffset
  * @param[out]     �����ǵ�ʱ�̣�ÿ����gyro_offset���û��?1,
  * @retval         none
  */
extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);

/**
  * @brief          get gyro zero drift from flash
  * @param[in]      cali_scale:scale, default 1.0
  * @param[in]      cali_offset:zero drift, 
  * @retval         none
  */
/**
  * @brief          У׼���������ã�����flash���������ط�����У׼ֵ
  * @param[in]      �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[in]      �����ǵ���Ư
  * @retval         none
  */
extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3]);

/**
  * @brief          get the quat
  * @param[in]      none
  * @retval         the point of INS_quat
  */
/**
  * @brief          ��ȡ��Ԫ��
  * @param[in]      none
  * @retval         INS_quat��ָ��
  */
extern const fp32 *get_INS_quat_point(void);


/**
  * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit rad
  * @param[in]      none
  * @retval         the point of INS_angle
  */
/**
  * @brief          ��ȡŷ����, 0:yaw, 1:pitch, 2:roll ��λ rad
  * @param[in]      none
  * @retval         INS_angle��ָ��
  */
extern const fp32 *get_INS_angle_point(void);


/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis,unit rad/s
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ rad/s
  * @param[in]      none
  * @retval         INS_gyro��ָ��
  */
extern const fp32 *get_gyro_data_point(void);


/**
  * @brief          get aceel, 0:x-axis, 1:y-axis, 2:roll-axis unit m/s2
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ m/s2
  * @param[in]      none
  * @retval         INS_gyro��ָ��
  */
extern const fp32 *get_accel_data_point(void);

/**
  * @brief          get mag, 0:x-axis, 1:y-axis, 2:roll-axis unit ut
  * @param[in]      none
  * @retval         the point of INS_mag
  */
/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ ut
  * @param[in]      none
  * @retval         INS_mag��ָ��
  */
extern const fp32 *get_mag_data_point(void);

#endif
