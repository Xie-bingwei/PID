#include "PID.h"

void Pid_Init(PID* pp)
{
    *pp = {0, 0, 0, 0, 0, 0, 0};
}

// actual：实际值  dErr：微分   Err：偏差
// 积分和微分在离散化模型中可看为加和减
double Pid_Calc(PID* pp, double actual) {
    double dErr, Err;
    Err = pp->target - actual; // 偏差 = 目标值 - 实际值
    pp->errSum += Err; // 误差积分 = 对偏差的累加
    dErr = pp->lastErr - pp->prevErr; // 微分 = 最后一次误差 - 上一次误差
    pp->prevErr = pp->lastErr;
    pp->lastErr = Err;         // 这两行为对误差的更新
    return (pp->Kp * Err + pp->Ki * pp->errSum + pp->Kd * dErr);
}
