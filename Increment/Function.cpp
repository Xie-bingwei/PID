#include "Pid.h"

void PID_Init(PID* pid, float Kp, float Ki, float Kd)
{
    pid->kp = Kp;
    pid->ki = Ki;
    pid->kd = Kd;
    pid->Target = 0;
    pid->prevActual = 0;
    pid->Err = 0;
    pid->lastErr = 0;
    pid->prevErr = 0;
}

float PID_Calc(float inputPoint, PID* pp)
{
    pp->Err = pp->Target - inputPoint;
    double calc = pp->kp*(pp->Err-(pp->lastErr)+pp->ki*pp->Err+(pp->kd*((pp->Err)-2*(pp->lastErr)+(pp->prevErr)))) + pp->prevActual;
    pp->prevErr = pp->lastErr;
    pp->lastErr = pp->Err;
    return calc;
}
