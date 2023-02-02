#ifndef INCREMENTALPID_PID_H
#define INCREMENTALPID_PID_H

typedef struct
{
    float kp ;		//比例系数
    float ki ;		//积分系数
    float kd ;	    //微分系数
    float Target;
    float prevActual;	//上一个误差值
    double Err; //当前误差
    double lastErr; // 上一次误差
    double prevErr;//上上次误差
}PID;

void PID_Init(PID* pid, float Kp, float Ki, float Kd);

float PID_Calc(float inputPoint, PID* pid);

#endif //INCREMENTALPID_PID_H
