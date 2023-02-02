#ifndef FUZZY_PID_MINE__FUZZY_H
#define FUZZY_PID_MINE__FUZZY_H

typedef struct
{
    float target;
    float err_prev; //上上一次误差
    float err_last; // 最后一次误差
    float err_sum;
    //定义划分区域个数
    const int area_num = 8;
    //定义隶属值
    float err_subordination_values[7] = {-3, -2, -1, 0, 1, 2, 3}; //误差
    float errc_subordination_values[7] = {-3, -2, -1, 0, 1, 2, 3}; //误差变化率
    float Kp_subordination_values[7] = {-3, -2, -1, 0, 1, 2, 3};
    float Ki_subordination_values[7] = {-3, -2, -1, 0, 1, 2, 3};
    float Kd_subordination_values[7] = {-3, -2, -1, 0, 1, 2, 3};
    //定义参数
    float Kp;
    float Ki;
    float Kd;
    //定义论域值
    float domain_value_Kp; // Kp对应的论域值，以下同理
    float domain_value_Ki;
    float domain_value_Kd;
    float domain_value_err;
    float domain_value_errc;
    //定义输出值
    float output_value_Kp; // Kp最终的输入值
    float output_value_Ki;
    float output_value_Kd;
    //定义参数的隶属度及其在规则表中的索引(即数组的下标)
    float membership_err[2];
    float membership_errc[2];
    int index_err[2];
    int index_errc[2];
    //定义各个索引对应隶属度
    float membership_sum_Kp[7];
    float membership_sum_Ki[7];
    float membership_sum_Kd[7];
    //建立规则表
    int NB = -3, NM = -2, NS = -1, ZO = 0, PS = 1, PM = 2, PB = 3;
    int rule_list_Kp[7][7] = {
            {PB, PB, PM, PM, PS, ZO, ZO},
            {PB, PB, PM, PS, PS, ZO, NS},
            {PM, PM, PM, PS, ZO, NS, NS},
            {PM, PM, PS, ZO, NS, NM, NM},
            {PS, ZO, NS, NM, NM, NM, NB},
            {ZO, ZO, NM, NM, NM, NB, NB}
    };
    int rule_list_Ki[7][7] = {
            {NB, NB, NM, NM, NS, ZO, ZO},
            {NB, NB, NM, NS, NS, ZO, ZO},
            {NB, NM, NS, NS, ZO, PS, PS},
            {NM, NM, NS, ZO, PS, PM, PM},
            {NM, NS, ZO, PS, PS, PM, PB},
            {ZO, ZO, PS, PS, PM, PB, PB},
            {ZO, ZO, PS, PM, PM, PB, PB}
    };
    int rule_list_Kd[7][7] = {
            {PS, NS, NB, NB, NB, NM, PS},
            {PS, NS, NB, NM, NM, NS, ZO},
            {ZO, NS, NM, NM, NS, NS, ZO},
            {ZO, NS, NS, NS, NS, NS, ZO},
            {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
            {PB, NS, PS, PS, PS, PS, PB},
            {PB, PM, PM, PM, PS, PS, PB}
    };
}FuzzyPid;

void PID_Init(FuzzyPid* pp);
void Get_membership(FuzzyPid* pp, float err, float errc); //得到隶属度
void Get_membership_sum(FuzzyPid* pp); //得到对应区间隶属度
void Get_domain_values(FuzzyPid* pp); //得到论域值
float FuzzyPID_calc(FuzzyPid* pp, float actual, float err_max, float err_min, float errc_max, float errc_min, float Kp_max, float Kp_min, float Ki_max, float Ki_min, float Kd_max, float Kd_min); //PID计算函数
float Interval_mapping_err(float e_max, float e_min, float e); //err 和 errc的区间映射函数
float Interval_mapping_Kx(float k_max, float k_min, float k);  // Kp,Ki,Kd的区间映射函数

#endif //FUZZY_PID_MINE__FUZZY_H
