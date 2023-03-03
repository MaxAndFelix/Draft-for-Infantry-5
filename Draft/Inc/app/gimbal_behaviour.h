#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "struct_typedef.h"
#include "gimbal_ctrl.h"

typedef enum
{
  GIMBAL_ZERO_FORCE = 0, 
  GIMBAL_INIT,           
  GIMBAL_CALI,           
  GIMBAL_ABSOLUTE_ANGLE, 
  GIMBAL_RELATIVE_ANGLE, 
  GIMBAL_MOTIONLESS,     
} gimbal_behaviour_e;


extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set); //被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set);//云台行为控制，根据不同行为采用不同控制函数
extern bool_t gimbal_cmd_to_chassis_stop(void); //云台在某些行为下，需要底盘不动
extern bool_t gimbal_cmd_to_shoot_stop(void); //云台在某些行为下，需要射击停止

#endif
