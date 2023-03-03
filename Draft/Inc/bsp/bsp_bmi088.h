#ifndef BSP_BMI088_H
#define BSP_BMI088_H
#include "struct_typedef.h"
#include "main.h"
#include "spi.h"
#include "cmsis_os.h"
#include "bsp_bmi088reg.h"
#include <stm32f4xx_hal.h>

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 1

#define BMI088_ACCEL_RANGE_3G
#define BMI088_GYRO_RANGE_2000

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

#define BMI088_ACCEL_NS_L() HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET)
#define BMI088_ACCEL_NS_H() HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET)
#define BMI088_GYRO_NS_L() HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET)
#define BMI088_GYRO_NS_H() HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET)

extern uint8_t BMI088_help_read_write_byte(uint8_t txdata);
extern void BMI088_help_write_single_reg(uint8_t reg, uint8_t data);
extern void BMI088_help_read_single_reg(uint8_t reg, uint8_t *return_data);
extern void BMI088_help_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#define BMI088_accel_write_single_reg(reg, data)     \
    {                                                \
        BMI088_ACCEL_NS_L();                         \
        BMI088_help_write_single_reg((reg), (data)); \
        BMI088_ACCEL_NS_H();                         \
    }
#define BMI088_accel_read_single_reg(reg, data)     \
    {                                               \
        BMI088_ACCEL_NS_L();                        \
        BMI088_help_read_write_byte((reg) | 0x80);  \
        BMI088_help_read_write_byte(0x55);          \
        (data) = BMI088_help_read_write_byte(0x55); \
        BMI088_ACCEL_NS_H();                        \
    }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_help_read_write_byte((reg) | 0x80); \
        BMI088_help_read_muli_reg(reg, data, len); \
        BMI088_ACCEL_NS_H();                       \
    }
#define BMI088_gyro_write_single_reg(reg, data)      \
    {                                                \
        BMI088_GYRO_NS_L();                          \
        BMI088_help_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                          \
    }
#define BMI088_gyro_read_single_reg(reg, data)       \
    {                                                \
        BMI088_GYRO_NS_L();                          \
        BMI088_help_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                          \
    }
#define BMI088_gyro_read_muli_reg(reg, data, len)        \
    {                                                    \
        BMI088_GYRO_NS_L();                              \
        BMI088_help_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                              \
    }

typedef struct __packed BMI088_RAW_DATA
{
    uint8_t status;
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
} BMI088_Raw_Data_T;

typedef struct BMI088_REAL_DATA
{
    uint8_t status;
    fp32 accel[3];
    fp32 temp;
    fp32 gyro[3];
    fp32 time;
} BMI088_Real_Data_T;

enum
{
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF,
};

extern uint8_t BMI088_help_read_write_byte(uint8_t reg);
extern uint8_t BMI088_init(void);
extern void BMI088_func_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate);
extern void BMI088_func_gyro_read_over(uint8_t *rx_buf, fp32 gyro[3]);
extern void BMI088_func_accel_read_over(uint8_t *rx_buf, fp32 accel[3], fp32 *time);
extern void BMI088_func_temperature_read_over(uint8_t *rx_buf, fp32 *temperate);
#endif