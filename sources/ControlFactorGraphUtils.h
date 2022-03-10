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

pair<VectorDynamic, VectorDynamic> ExtractResults(const Values &result, TaskSet &tasks)
{
    VectorDynamic periods = GenerateVectorDynamic(tasks.size());
    VectorDynamic rta = GenerateVectorDynamic(tasks.size());
    for (uint i = 0; i < tasks.size(); i++)
    {
        if (result.exists(GenerateControlKey(i, "period")))
        {
            periods(i, 0) = result.at<VectorDynamic>(GenerateControlKey(i, "period"))(0, 0);
        }
        else
        {
            periods(i, 0) = tasks[i].period;
        }
        rta(i, 0) = result.at<VectorDynamic>(GenerateControlKey(i, "response"))(0, 0);
    }
    return make_pair(periods, rta);
}
