#include "Fuzzy.h"

void PID_Init(FuzzyPid* pp)
{
    pp->Kp = 0;
    pp->Ki = 0;
    pp->Kd = 0;
    pp->domain_value_Kp = 0;
    pp->domain_value_Ki = 0;
    pp->domain_value_Kd = 0;
    pp->err_sum = 0;
    pp->target = 600;
}

void Get_membership(FuzzyPid* pp, float err, float errc)
{
    if(err > pp->err_subordination_values[0] && err < pp->err_subordination_values[6])
    {
        for(int i = 0; i < pp->area_num - 2; i++)
        {
            if(err >= pp->err_subordination_values[i] && err <= pp->err_subordination_values[i + 1])
            {
                pp->membership_err[0] = -(err - pp->err_subordination_values[i + 1])
                        / (pp->err_subordination_values[i + 1] - pp->err_subordination_values[i]);
                pp->membership_err[1] = 1 + (err - pp->err_subordination_values[i + 1])
                        / (pp->err_subordination_values[i + 1] - pp->err_subordination_values[i]);
                pp->index_err[0] = i;
                pp->index_err[1] = i + 1;
                break;
            }
         }
    }
    else
    {
        if(err <= pp->err_subordination_values[0])
        {
            pp->membership_err[0] = 1;
            pp->membership_err[1] = 0;
            pp->index_err[0] = 0;
            pp->index_err[1] = -1;
        }
        else if(err > pp->err_subordination_values[6])
        {
            pp->membership_err[0] = 1;
            pp->membership_err[1] = 0;
            pp->index_err[0] = 6;
            pp->index_err[1] = -1;
        }
    }
    if(errc > pp->errc_subordination_values[0] && errc < pp->errc_subordination_values[6])
    {
        for(int i = 0; i < pp->area_num - 2; i++)
        {
            if(errc >= pp->errc_subordination_values[i] && errc <= pp->errc_subordination_values[i + 1])
            {
                pp->membership_errc[0] = -(errc - pp->errc_subordination_values[i + 1])
                        / (pp->errc_subordination_values[i + 1] - pp->errc_subordination_values[i]);
                pp->membership_errc[1] = 1 + (errc - pp->errc_subordination_values[i + 1])
                        / (pp->errc_subordination_values[i + 1] - pp->errc_subordination_values[i]);
                pp->index_errc[0] = i;
                pp->index_errc[1] = i + 1;
                break;
            }
        }
    }
    else
    {
        if(errc <= pp->errc_subordination_values[0])
        {
            pp->membership_errc[0] = 1;
            pp->membership_errc[1] = 0;
            pp->index_errc[0] = 0;
            pp->index_errc[1] = -1;
        }
        else if(errc >= pp->errc_subordination_values[6])
        {
            pp->membership_errc[0] = 1;
            pp->membership_errc[1] = 0;
            pp->index_errc[0] = 6;
            pp->index_errc[1] = -1;
        }
    }
}

void Get_membership_sum(FuzzyPid* pp)
{
    for(int i = 0; i < pp->area_num - 1; i++)
    {
        pp->membership_sum_Kp[i] = 0;
        pp->membership_sum_Ki[i] = 0;
        pp->membership_sum_Kd[i] = 0;
    }
    for(int i = 0; i < 2; i++)
    {
        if(pp->index_err[i] == -1)
            continue;
        for(int j = 0; j < 2; j++)
        {
            if(pp->index_errc[j] != -1)
            {
                int indexKp = pp->rule_list_Kp[pp->index_err[i]][pp->index_errc[j]] + 3;
                int indexKi = pp->rule_list_Ki[pp->index_err[i]][pp->index_errc[j]] + 3;
                int indexKd = pp->rule_list_Kd[pp->index_err[i]][pp->index_errc[j]] + 3;
                pp->membership_sum_Kp[indexKp] = pp->membership_sum_Kp[indexKp] + (pp->membership_err[i] * pp->membership_errc[j]);
                pp->membership_sum_Ki[indexKi] = pp->membership_sum_Ki[indexKi] + (pp->membership_err[i] * pp->membership_errc[j]);
                pp->membership_sum_Kd[indexKd] = pp->membership_sum_Kd[indexKd] + (pp->membership_err[i] * pp->membership_errc[j]);
            }
            else
                continue;
        }
    }
}

void Get_domain_values(FuzzyPid* pp)
{
    for(int i = 0; i < pp->area_num - 1; i++)
    {
        pp->domain_value_Kp += pp->Kp_subordination_values[i] * pp->membership_sum_Kp[i];
        pp->domain_value_Ki += pp->Ki_subordination_values[i] * pp->membership_sum_Ki[i];
        pp->domain_value_Kd += pp->Kd_subordination_values[i] * pp->membership_sum_Kd[i];
    }
}

float FuzzyPID_calc(FuzzyPid* pp, float actual, float err_max, float err_min, float errc_max, float errc_min, float Kp_max, float Kp_min, float Ki_max, float Ki_min, float Kd_max, float Kd_min)
{
    float err = pp->target - actual;
    float errc = err - pp->err_last;
    pp->err_sum += err;
    pp->domain_value_err = Interval_mapping_err(err_max, err_min, err);
    pp->domain_value_errc = Interval_mapping_err(errc_max, errc_min, errc);
    Get_membership(pp, pp->domain_value_err, pp->domain_value_errc);
    Get_membership_sum(pp);
    Get_domain_values(pp);
    pp->output_value_Kp = Interval_mapping_Kx(Kp_max, Kp_min, pp->domain_value_Kp);
    pp->output_value_Ki = Interval_mapping_Kx(Ki_max, Ki_min, pp->domain_value_Ki);
    pp->output_value_Kd = Interval_mapping_Kx(Kd_max, Kd_min, pp->domain_value_Kd);
    pp->domain_value_Kp = 0, pp->domain_value_Ki = 0, pp->domain_value_Kd = 0;

    pp->Kp += pp->output_value_Kp;
    pp->Ki += pp->output_value_Ki;
    pp->Kd += pp->output_value_Kd;
    if(pp->Kp < 0) pp->Kp = 0;
    if(pp->Ki < 0) pp->Ki = 0;
    if(pp->Kd < 0) pp->Kd = 0;
    pp->output_value_Kp = 0, pp->output_value_Ki = 0, pp->output_value_Kd = 0;

    return pp->Kp * (err - pp->err_last) + pp->Ki * err + pp->Kd * (err - 2 * pp->err_last + pp->err_prev);
}

float Interval_mapping_err(float e_max, float e_min, float e)
{
    float values = 6.0 * (e - e_min) / (e_max - e_min) - 3;
    return values;
}

float Interval_mapping_Kx(float k_max, float k_min, float k)
{
    float Kx = (k_max - k_min) * (k + 3) / 6 + k_min;
    return Kx;
}
