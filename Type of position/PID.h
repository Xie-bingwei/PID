#ifndef PID_PID_H
#define PID_PID_H

typedef struct
{
    double Kp;
    double Ki;
    double Kd;
    double target;  // 目标值
    double lastErr;  //最后一次误差
    double prevErr; // 上一次误差
    double errSum; // 误差积分
}PID;

void Pid_Init(PID* pp);
double Pid_Calc(PID* pp, double actual);

#endif //PID_PID_H
