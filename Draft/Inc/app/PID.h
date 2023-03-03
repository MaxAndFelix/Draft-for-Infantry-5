#ifndef __PID_H
#define __PID_H
#include "main.h"
#include "struct_typedef.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};
typedef struct
{
    uint8_t mode;
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 i_max;
    fp32 out_max;

    fp32 ref;
    fp32 fdb;

    fp32 out;
    fp32 p_out;
    fp32 i_out;
    fp32 d_out;
    fp32 Dbuf[3];
    fp32 err[3];

} pid_type_def;

void pid_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
float pid_calc(pid_type_def *pid, float ref, float fdb);
#endif
