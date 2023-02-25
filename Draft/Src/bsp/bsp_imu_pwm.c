#include "bsp_imu_pwm.h"
#include "main.h"
#include "stm32f4xx_hal_tim.h"

extern TIM_HandleTypeDef htim10;
void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}
