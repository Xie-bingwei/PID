#include <iostream>
#include "PID.h"
using namespace std;

int main() {
    PID pid;
    Pid_Init(&pid);
    pid.target = 600.000;
    pid.Kp = 0.050, pid.Ki = 0.30, pid.Kd = 0.300; // 在实际中可以定义函数随时更改他们的值
    double actual = 0;
    for(int i = 0; i < 100; i++)
    {
        actual = Pid_Calc(&pid, actual); // 对传入值的更新
        cout<<"Times: "<< i + 1<< "\t"<< "Target: "<< pid.target<<"\t"<<"Actual: "<<actual<< endl;
    }
    return 0;
}
