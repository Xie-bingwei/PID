#include <iostream>
#include "Fuzzy.h"

int main() {
    FuzzyPid pp;
    PID_Init(&pp);
    float actual = 0;
    float err_max =1000, err_min = -1000;
    float errc_max = 800, errc_min = -800;
    float kp_max =100, kp_min = -100;
    float ki_max = 0.1, ki_min = -0.1;
    float kd_max = 0.01, kd_min = -0.01;
    float err, errc;
    err =pp.target - actual;
    errc = err - pp.err_last;
    for(int i = 0; i < 100; i++)
    {
        float value;
        value = FuzzyPID_calc(&pp, actual, err_max, err_min, errc_max, errc_min, kp_max, kp_min,ki_max,ki_min,kd_max,kd_min);
        actual += value;
        pp.err_prev = pp.err_last;
        pp.err_last = err;
        err = pp.target - actual;
        errc= err - pp.err_last;
        std::cout << "Times:" << i << "\t" << "Target:" << pp.target << "\t" << "Actual:" << actual << std::endl;
    }
    return 0;
}
