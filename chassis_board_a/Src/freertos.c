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
osThreadId communicationHandle;
osThreadId can_6020_yawHandle;
osThreadId can_3508Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void communication_task(void const * argument);
void gimbal_yaw_task(void const * argument);
void chassis_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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

  /* definition and creation of communication */
  osThreadDef(communication, communication_task, osPriorityIdle, 0, 128);
  communicationHandle = osThreadCreate(osThread(communication), NULL);

  /* definition and creation of can_6020_yaw */
  osThreadDef(can_6020_yaw, gimbal_yaw_task, osPriorityIdle, 0, 128);
  can_6020_yawHandle = osThreadCreate(osThread(can_6020_yaw), NULL);

  /* definition and creation of can_3508 */
  osThreadDef(can_3508, chassis_task, osPriorityIdle, 0, 128);
  can_3508Handle = osThreadCreate(osThread(can_3508), NULL);

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_communication_task */
/**
* @brief Function implementing the can1_com thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_communication_task */
void communication_task(void const * argument)
{
  /* USER CODE BEGIN communication_task */
  //板间通讯在can接收回调函数中实�? HAL_CAN_RxFifo0MsgPendingCallback
  /* Infinite loop */
  motor_info[0].set_voltage = 0;
  motor_info[1].set_voltage = 0;
  motor_info[2].set_voltage = 0;
  motor_info[3].set_voltage = 0; //防止bug
  motor_info[4].set_voltage = 0;
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

/* USER CODE BEGIN Header_gimbal_yaw_task */
/**
* @brief Function implementing the can_6020_yaw thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_yaw_task */
void gimbal_yaw_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_yaw_task */
  /* Infinite loop */
  pid_init(&motor_pid[4], 40, 3, 0, 30000, 30000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 30000    
  for(;;)//�?单的yaw轴随rc通道0转动
  {
    if(can_flag==1)
    {
      if (rc_ctrl.rc.ch[0] >= 974 && rc_ctrl.rc.ch[0] <= 1074) //回中
      {
        if (motor_info[4].rotor_speed > 10 || motor_info[4].rotor_speed < -10) //减震(?)
        {
          target_speed[4] = 0;
          motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
        }
        else
        {
          motor_info[4].set_voltage = 0;
        }
      }else
      {
        if(rc_ctrl.rc.ch[0]>=364&&rc_ctrl.rc.ch[0]<=1684)
        {
          target_speed[4] = (rc_ctrl.rc.ch[0] - 1024) / 660 * 15;
          motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
        }
      }
      set_motor_voltage1(1, motor_info[4].set_voltage, motor_info[5].set_voltage, motor_info[6].set_voltage, 0);
      //set_motor_voltage1(1, 30, 30, 30, 0);
    }
    osDelay(1);
  }
  /* USER CODE END gimbal_yaw_task */
}

/* USER CODE BEGIN Header_chassis_task */
/**
* @brief Function implementing the can_3508_down thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
void chassis_task(void const * argument)
{
  /* USER CODE BEGIN chassis_task */
  //驱动3508麦轮底盘，can的接收频率明显比uart�?(can红灯闪烁频率>uart蓝灯闪烁频率)
  /* Infinite loop */
  for (uint8_t i = 0; i < 4; i++) //先将pid赋初�?
  {
    pid_init(&motor_pid[i], 40, 3, 0, 16384, 16384); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
  }                                                   // P高了就会抖动，返回�?�溢出，超过80电机有移动就会抖动，超过100直接�?�?(除非你设定的速度�?>=320)

  for (;;)
  {
    for (uint8_t i = 0; i < 4; i++)
    {                                                                                                  //�?大�?�为482*19=9158
      motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed[i], motor_info[i].rotor_speed); //解算PID输出
    }                                                                                                  //经测试，float的�?�赋给int时，会直接舍去小数点后面的数(不是四舍五入)
    if (can_flag == 1)                                                                                 // can通信成功,->35```````````````````````08
    {

      if ((rc_ctrl.rc.ch[2] >= 974 && rc_ctrl.rc.ch[2] <= 1074) && ((rc_ctrl.rc.ch[3] >= 974) && (rc_ctrl.rc.ch[3] <= 1074)) && (rc_ctrl.rc.ch[4] <= 1074) && (rc_ctrl.rc.ch[4] >= 974))
      {
        for (int i = 0; i < 4; i++) //减�??
        {
          if (motor_info[i].rotor_speed > 720 || motor_info[i].rotor_speed < -720)
          {
            target_speed[i] = 0;
            motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed[i], motor_info[i].rotor_speed);
          }
          else
          {
            motor_info[i].set_voltage = 0;
          }
        }
      }
      else
      {
        if (rc_ctrl.rc.ch[4] > 1074)
        {
          target_curl = sqrt((rc_ctrl.rc.ch[4] - 1024) * (rc_ctrl.rc.ch[4] - 1024)) / 660;
        } //不会强制转换，但用这个可以实�?
        else if (rc_ctrl.rc.ch[4] < 974)
        {
          target_curl = -(sqrt((rc_ctrl.rc.ch[4] - 1024) * (rc_ctrl.rc.ch[4] - 1024)) / 660);
        }
        else
        {
          target_curl = 0;
        }
        target_curl = target_curl * 9158;
        int16_t target_curl_int = target_curl;

        r = sqrt((rc_ctrl.rc.ch[3] - 1024) * (rc_ctrl.rc.ch[3] - 1024) + (rc_ctrl.rc.ch[2] - 1024) * (rc_ctrl.rc.ch[2] - 1024));
        sin_sita = (rc_ctrl.rc.ch[3] - 1024) / r;
        cos_sita = (rc_ctrl.rc.ch[2] - 1024) / r;
        target_v = (r / 660) * 9158;

        if (target_curl == 0)
        {
          target_speed[3] = (0.707 * target_v * (sin_sita - cos_sita)) / 2;
          target_speed[1] = -(0.707 * target_v * (sin_sita - cos_sita)) / 2;
          target_speed[0] = (0.707 * target_v * (sin_sita + cos_sita)) / 2;
          target_speed[2] = -(0.707 * target_v * (sin_sita + cos_sita)) / 2; //保护不旋转只移动时的底盘功率上限
        }
        else
        {
          target_int1 = (0.707 * target_v * (sin_sita - cos_sita)) / 2; //经测验，double和float类型可以做乘除，做加减会出现NAN,因此转换成int做加减法，注意不要使用uint（无符号型）
          target_int2 = (0.707 * target_v * (sin_sita + cos_sita)) / 2;

          target_speed[3] = (target_int1 - target_curl_int) / 2;
          target_speed[1] = (-target_int1 - target_curl_int) / 2;
          target_speed[0] = (target_int2 - target_curl_int) / 2;
          target_speed[2] = (-target_int2 - target_curl_int) / 2; //不�?2会越�?
        }
      }
    }
    else //连接未成�?
    {
      motor_info[0].set_voltage = 0;
      motor_info[1].set_voltage = 0;
      motor_info[2].set_voltage = 0;
      motor_info[3].set_voltage = 0;
    }
    
    set_motor_voltage(0,
                      motor_info[0].set_voltage,
                      motor_info[1].set_voltage,
                      motor_info[2].set_voltage,
                      motor_info[3].set_voltage);
    osDelay(1);
  }
  /* USER CODE END chassis_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
