#ifndef __PID_H
#define __PID_H
#include "main.h"
#include "struct_typedef.h"
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);

float pid_calc(pid_struct_t *pid, float ref, float fdb);
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};
typedef struct
{
    uint8_t mode;
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;
    fp32 max_iout;

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];
    fp32 error[3];

} pid_type_def;
#endif
