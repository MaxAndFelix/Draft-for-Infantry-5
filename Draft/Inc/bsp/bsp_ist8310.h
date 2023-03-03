#ifndef BSP_IST8310_H
#define BSP_IST8310_H
#include "struct_typedef.h"
#include "main.h"
#include "cmsis_os.h"
#define IST8310_IIC_ADDRESS 0x0E
#define IST8310_DATA_READY_BIT 2
#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40
#define MAG_SEN 0.3f //raw int16 data change to uT unit.
#define IST8310_WHO_AM_I 0x00       //ist8310 "who am I " 
#define IST8310_WHO_AM_I_VALUE 0x10 //device ID
#define IST8310_WRITE_REG_NUM 4 



typedef struct IST8310_Real_Data_T
{
    uint8_t status;
    fp32 mag[3];
} IST8310_Real_Data_T;
extern I2C_HandleTypeDef hi2c3;
extern uint8_t IST8310_func_Init(void); // 初始化函数
extern void IST8310_func_read_over(uint8_t *status_buf, IST8310_Real_Data_T *ist8310_real_data);
extern void ist8310_func_read_mag(fp32 mag[3]);

#endif
