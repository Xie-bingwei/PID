#include <iostream>
#include "Pid.h"
using namespace std;

int main() {
    PID pp;
    PID_Init(&pp, 0.5, 0.3, 0.2);
    pp.Target = 600;
    float actual = 0;
    for(int i = 0; i < 100; i++)
    {
        actual = PID_Calc(actual, &pp);
        cout<< "Times: "<< i + 1<< "\t"<< "Target: "<< pp.Target<< "\t"<< "Actual: "<<  actual<< endl;
        pp.Err = pp.Target - actual;
        pp.prevActual = actual;
    }
    return 0;
}

