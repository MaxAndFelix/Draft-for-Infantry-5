#include "bsp_ist8310.h"

// 第一列:IST8310的寄存器
// 第二列:需要写入的寄存器值
// 第三列:返回的错误码
static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x08, 0x01},  // enalbe interrupt  and low pin polarity.开启中断，并且设置低电平
    {0x41, 0x09, 0x02},  // average 2 times.平均采样两次
    {0x42, 0xC0, 0x03},  // must be 0xC0. 必须是0xC0
    {0x0A, 0x0B, 0x04}}; // 200Hz output rate.200Hz输出频率

uint8_t IST8310_help_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res = 0;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 10);
    return res;
}
void IST8310_help_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}
void IST8310_help_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10);
}
void HAL_Delay_us(uint16_t us)
{
    uint32_t ticks = 0;
    uint32_t told = 0, tnow = 0, tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 72;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

uint8_t IST8310_func_Init(void)
{
    static const uint8_t wait_time = 150;
    static const uint8_t sleepTime = 50;
    uint8_t res = 0;
    uint8_t writeNum = 0;
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
    osDelay(sleepTime);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
    osDelay(sleepTime);
    res = IST8310_help_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }
    // set mpu6500 sonsor config and check
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
        IST8310_help_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
        HAL_Delay_us(wait_time);
        res = IST8310_help_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
        HAL_Delay_us(wait_time);
        if (res != ist8310_write_reg_data_error[writeNum][1])
        {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }
    return IST8310_NO_ERROR;
}

void IST8310_func_read_over(uint8_t *status_buf, IST8310_Real_Data_T *ist8310_real_data)
{

    if (status_buf[0] & 0x01)
    {
        int16_t temp_ist8310_data = 0;
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;
        temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
    }
    else
    {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

void ist8310_func_read_mag(fp32 mag[3])
{
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
    IST8310_help_IIC_read_muli_reg(0x03, buf, 6);

    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2] = MAG_SEN * temp_ist8310_data;
}