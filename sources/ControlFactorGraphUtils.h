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
    double res = 0;
    RTA_LL r(tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    for (uint i = 0; i < tasks.size(); i++)
    {
        res += coeff(i * 2, 0) * tasks[i].period;
        res += coeff(i * 2 + 1, 0) * rta(i, 0);
    }
    return res;
}

void FindEliminatedVariables(TaskSet &tasks, std::vector<bool> &maskForElimination, double disturb = 1e0)
{
    RTA_LL r(tasks);
    VectorDynamic rtaBase = r.ResponseTimeOfTaskSet();
    for (uint i = 0; i < tasks.size(); i++)
    {
        tasks[i].period -= disturb;
        RTA_LL r1(tasks);
        VectorDynamic rtaCurr = r1.ResponseTimeOfTaskSet();
        if ((rtaBase - rtaCurr).array().abs().maxCoeff() >= disturb)
        // TODO: more analytic way
        {
            maskForElimination[i] = true;
        }
        tasks[i].period += disturb;
    }
    for (auto a : maskForElimination)
        cout << a << ", ";
    cout << endl;
}