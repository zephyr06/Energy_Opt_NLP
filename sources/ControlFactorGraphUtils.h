#pragma once
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>

/**
 * @brief generate keys for factor graph modeling the control problem
 * keys' attributes can be extracted via key.chr(). key.index()
 * 
 * @param idtask 
 * @param type 
 * @return gtsam::Symbol 
 */
inline gtsam::Symbol GenerateControlKey(int idtask, string type)
{
    if (type == "period")
    {
        return gtsam::Symbol('t', idtask);
    }
    else if (type == "response")
    {
        if (idtask == 4)
        {
            int a = 1;
        }
        return gtsam::Symbol('r', idtask);
    }
    else
    {
        CoutError("Unrecognized type in GenerateControlKey");
        gtsam::Symbol key('a', idtask);
        return key;
    }
}

double RealObj(TaskSet &tasks, VectorDynamic coeff)
{
    BeginTimer(__func__);
    double res = 0;
    RTA_LL r(tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    for (uint i = 0; i < tasks.size(); i++)
    {
        res += coeff(i * 2, 0) * tasks[i].period;
        res += coeff(i * 2 + 1, 0) * rta(i, 0);
    }
    EndTimer(__func__);
    return res;
}

/**
 * @brief perform customized quotient operation for two double type numbers
 * return result is min_k |a - k b|, where k is an integer, a & b >0;
 * 
 * Brute force solution, find minimum remainder by finding minimum rem=(a-=b)
 * @param a 
 * @param b 
 * @return double 
 */
double QuotientDouble(double a, double b)
{
    double remainder = a - int(a / b) * b;
    return min(remainder, abs(remainder - b));
}

template <typename T>
bool Equals(std::vector<T> &v1, std::vector<T> &v2)
{
    if (v1.size() != v2.size())
    {
        return false;
    }
    for (uint i = 0; i < v1.size(); i++)
    {
        if (v1[i] != v2[i])
        {
            return false;
        }
    }
    return true;
}