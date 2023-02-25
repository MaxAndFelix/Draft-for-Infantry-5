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
#include "PID.h"
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId weopenHandle;
osThreadId can_6020_pitchHandle;
osThreadId communicationHandle;
osThreadId insHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void weopen_task(void const * argument);
void gimbal_pitch_task(void const * argument);
void communication_task(void const * argument);
void insTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of weopen */
  osThreadDef(weopen, weopen_task, osPriorityIdle, 0, 128);
  weopenHandle = osThreadCreate(osThread(weopen), NULL);

  /* definition and creation of can_6020_pitch */
  osThreadDef(can_6020_pitch, gimbal_pitch_task, osPriorityIdle, 0, 128);
  can_6020_pitchHandle = osThreadCreate(osThread(can_6020_pitch), NULL);

  /* definition and creation of communication */
  osThreadDef(communication, communication_task, osPriorityIdle, 0, 128);
  communicationHandle = osThreadCreate(osThread(communication), NULL);

  /* definition and creation of ins */
  osThreadDef(ins, insTask, osPriorityIdle, 0, 128);
  insHandle = osThreadCreate(osThread(ins), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  //HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_weopen_task */
/**
* @brief Function implementing the weopen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_weopen_task */
void weopen_task(void const * argument)
{
  /* USER CODE BEGIN weopen_task */
  /* Infinite loop */
  int flag=0;
  for (uint8_t i = 0; i < 4; i++) 
  {
    pid_init(&motor_pid[i], 40, 3, 0, 30000, 30000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 30000
  }    
  for(;;)
  {
    if (rc_ctrl.rc.s[1] == 2)
    {
      /*while (motor_info[2].torque_current!=-16864&&flag==0)
      {
        target_speed[2] = -50;
        motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);  //���ڷ�תת��
        set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage, motor_info[2].set_voltage,0);
      }*/
      flag=1;
      target_speed[0] = -5000;
      target_speed[1] = 5000;
      target_speed[2] = 400;
      motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
      motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
      motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
      set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage, motor_info[2].set_voltage,0);
    }else
    {
      flag=0;
      target_speed[0] = 0;
      target_speed[1] = 0;
      target_speed[2] = 0;
      motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
      motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
      motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
      set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage, motor_info[2].set_voltage,0);
    }
    osDelay(1);
  }
  /* USER CODE END weopen_task */
}

/* USER CODE BEGIN Header_gimbal_pitch_task */
/**
* @brief Function implementing the can_6020_pitch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_pitch_task */
void gimbal_pitch_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_pitch_task */
  /* Infinite loop */
  pid_init(&motor_pid[6], 40, 3, 0, 30000, 30000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 30000  
  for(;;)
  {
    if(can_flag ==1)
    {
      if (rc_ctrl.rc.ch[1] >= 974 && rc_ctrl.rc.ch[1] <= 1074) //����
      {
        if (motor_info[6].rotor_speed > 10 || motor_info[6].rotor_speed < -10) //����(?)
        {
          target_speed[6] = 0;
          motor_info[6].set_voltage = pid_calc(&motor_pid[6], target_speed[6], motor_info[6].rotor_speed);
        }
        else
        {
          target_speed[6] = 0;
        }
      }else
      {
        if(rc_ctrl.rc.ch[1]>=364&&rc_ctrl.rc.ch[1]<=1684)
        {
          target_speed[6] = -((rc_ctrl.rc.ch[1] - 1024) / 660 * 8);
          motor_info[6].set_voltage = pid_calc(&motor_pid[6], target_speed[6], motor_info[6].rotor_speed);
        }
      }
      set_motor_voltage1(1,motor_info[4].set_voltage, motor_info[5].set_voltage, motor_info[6].set_voltage, 0);
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
void communication_task(void const * argument)
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

/* USER CODE BEGIN Header_insTask */
/**
* @brief Function implementing the ins thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_insTask */
void insTask(void const * argument)
{
  /* USER CODE BEGIN insTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END insTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
