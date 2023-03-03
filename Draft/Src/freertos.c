/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote_control.h"
#include "math.h"
#include "pid.h"
#include "info_proc.h"
#include "bsp_usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static const fp32 motor_PID[3] = {40, 3, 0};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for weapon */
osThreadId_t weaponHandle;
const osThreadAttr_t weapon_attributes = {
    .name = "weapon",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for can_6020_pitch */
osThreadId_t can_6020_pitchHandle;
const osThreadAttr_t can_6020_pitch_attributes = {
    .name = "can_6020_pitch",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for communication */
osThreadId_t communicationHandle;
const osThreadAttr_t communication_attributes = {
    .name = "communication",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for info_proc */
osThreadId_t info_procHandle;
const osThreadAttr_t info_proc_attributes = {
    .name = "info_proc",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void weapon_task(void *argument);
void gimbal_pitch_task(void *argument);
void communication_task(void *argument);
void Info_Proc(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of weapon */
  weaponHandle = osThreadNew(weapon_task, NULL, &weapon_attributes);

  /* creation of can_6020_pitch */
  can_6020_pitchHandle = osThreadNew(gimbal_pitch_task, NULL, &can_6020_pitch_attributes);

  /* creation of communication */
  communicationHandle = osThreadNew(communication_task, NULL, &communication_attributes);

  /* creation of info_proc */
  info_procHandle = osThreadNew(Info_Proc, NULL, &info_proc_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  // HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
  /* Infinite loop */

  for (;;)
  {
    if (imu_start_dma_flag)
    {
      UART_func_printf("0:");
      UART_func_printfloat(INS_angle[0]);
      UART_func_printf(" 1:");
      UART_func_printfloat(INS_angle[1]);
      UART_func_printf(" 2:");
      UART_func_printfloat(INS_angle[2]);
      UART_func_printf("\n");
      osDelay(1000);
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_weapon_task */
/**
 * @brief Function implementing the weapon thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_weapon_task */
void weapon_task(void *argument)
{
  /* USER CODE BEGIN weapon_task */
  /* Infinite loop */
  for (uint8_t i = 0; i < 4; i++)
  {
    pid_init(&motor_pid[i], PID_POSITION, motor_PID, 30000, 30000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 30000
  }
  for (;;)
  {
    if (rc_ctrl.rc.s[1] == 2)
    {
      // while (motor_info[6].torque_current!=16864)
      // {
      //   target_speed[2] = -40;
      //   motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);  //���ڷ�תת��
      //   set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage, motor_info[2].set_voltage,0);
      // }
      target_speed[0] = -5000;
      target_speed[1] = 5000;
      target_speed[2] = 300;
      motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
      motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
      motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
      set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage, motor_info[2].set_voltage, 0);
    }
    else
    {
      target_speed[0] = 0;
      target_speed[1] = 0;
      target_speed[2] = 0;
      motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
      motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
      motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
      set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage, motor_info[2].set_voltage, 0);
    }
    osDelay(1);
  }
  /* USER CODE END weapon_task */
}

/* USER CODE BEGIN Header_gimbal_pitch_task */
/**
 * @brief Function implementing the can_6020_pitch thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_gimbal_pitch_task */
void gimbal_pitch_task(void *argument)
{
  /* USER CODE BEGIN gimbal_pitch_task */
  /* Infinite loop */
  pid_init(&motor_pid[6], PID_POSITION, motor_PID, 30000, 30000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 30000
  for (;;)
  {
    if (can_flag == 1)
    {
      if (rc_ctrl.rc.ch[1] >= 974 && rc_ctrl.rc.ch[1] <= 1074) // ����
      {
        if (motor_info[6].rotor_speed > 10 || motor_info[6].rotor_speed < -10) // ����(?)
        {
          target_speed[6] = 0;
          motor_info[6].set_voltage = pid_calc(&motor_pid[6], target_speed[6], motor_info[6].rotor_speed);
        }
        else
        {
          target_speed[6] = 0;
        }
      }
      else
      {
        if (rc_ctrl.rc.ch[1] >= 364 && rc_ctrl.rc.ch[1] <= 1684)
        {
          target_speed[6] = -((rc_ctrl.rc.ch[1] - 1024) / 660 * 8);
          motor_info[6].set_voltage = pid_calc(&motor_pid[6], target_speed[6], motor_info[6].rotor_speed);
        }
      }
      set_motor_voltage1(1, motor_info[4].set_voltage, motor_info[5].set_voltage, motor_info[6].set_voltage, 0);
    }
    osDelay(1);
  }
  /* USER CODE END gimbal_pitch_task */
}

/* USER CODE BEGIN Header_communication_task */
/**
 * @brief Function implementing the communication thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_communication_task */
void communication_task(void *argument)
{
  /* USER CODE BEGIN communication_task */
  /* Infinite loop */
  motor_info[1].set_voltage = 0;
  motor_info[2].set_voltage = 0;
  motor_info[3].set_voltage = 0;
  motor_info[4].set_voltage = 0;
  motor_info[5].set_voltage = 0;
  motor_info[6].set_voltage = 0;
  motor_info[7].set_voltage = 0;
  for (;;)
  {
    if (rc_ctrl.rc.ch[3] > 300)
    {
      can_flag = 1;
      HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
    }
    osDelay(1);
  }
  /* USER CODE END communication_task */
}

/* USER CODE BEGIN Header_Info_Proc */
/**
 * @brief Function implementing the info_proc thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Info_Proc */
__weak void Info_Proc(void *argument)
{
  /* USER CODE BEGIN Info_Proc */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END Info_Proc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
