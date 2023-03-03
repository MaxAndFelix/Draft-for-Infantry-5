#include "main.h"
#include "PID.h"
extern CAN_HandleTypeDef hcan1;
void pid_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 out_max, fp32 i_max)
{
  if (pid == NULL || PID == NULL)
  {
    return;
  }
  pid->mode = mode;
  pid->kp = PID[0];
  pid->ki = PID[1];
  pid->kd = PID[2];
  pid->out_max = out_max;
  pid->i_max = i_max;
  pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->err[0] = pid->err[1] = pid->err[2] = pid->p_out = pid->i_out = pid->d_out = pid->out = 0.0f;
}

float pid_calc(pid_type_def *pid, fp32 ref, fp32 fdb) // ref是目标值,fdb是电机解码的速度返回值
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];         // err[1]是上一次计算出来的差值
  pid->err[0] = pid->ref - pid->fdb; // err[0]是这一次的预期速度和实际速度的差值,这两个值是可以是负数的

  pid->p_out = pid->kp * pid->err[0]; // 40 3 0是标准值，把这个加到watch1里面
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max); // 防止越界

  pid->out = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->out, -pid->out_max, pid->out_max); // 防止越界
  return pid->out;                                      // 电机返回的报文有转速和转矩电流，但是只能发电压值(-30000至30000)，有点抽象这个PID
}
