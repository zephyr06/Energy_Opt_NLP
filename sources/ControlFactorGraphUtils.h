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

pair<VectorDynamic, VectorDynamic> ExtractResults(const Values &result)
{
    VectorDynamic periods = GenerateVectorDynamic(result.size() / 2);
    VectorDynamic rta = GenerateVectorDynamic(result.size() / 2);
    for (uint i = 0; i < result.size() / 2; i++)
    {
        if (result.exists(GenerateControlKey(i, "period")))
        {
            periods(i, 0) = result.at<VectorDynamic>(GenerateControlKey(i, "period"))(0, 0);
            rta(i, 0) = result.at<VectorDynamic>(GenerateControlKey(i, "response"))(0, 0);
        }
        else
        {
            CoutError("Request non-existed key in ExtractPeriod" + to_string(i));
        }
    }
    return make_pair(periods, rta);
}

void PrintControlValues(const Values &result)
{
    cout << "Period vector is " << endl
         << ExtractResults(result).first << endl;
    cout << "RTA vector is " << endl
         << ExtractResults(result).second << endl;
}
