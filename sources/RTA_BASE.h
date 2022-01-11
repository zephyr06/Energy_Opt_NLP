#pragma once

#include "Tasks.h"
#include "Declaration.h"
/**
 * @brief All customized TaskType must inherit from Task in Tasks.h
 * 
 */

/**
 * @brief RTA_BASE encapsulate all the interafaces required
 *  by Energy_optimization.
 * All kinds of RTA should inherit 
 * from RTA_BASE and implement its virtual function
 * 
 */

template <class TaskType>
class RTA_BASE
{
public:
    typedef std::vector<TaskType> TaskSetType;

    /**
     * @brief 
     * 
     * @param tasks inverse priority ordered
     * @param index 
     * @return double 
     */
    static double RTA_Common_Warm(double beginTime, const TaskSetType &tasks, int index);
    static double RTA_Common(const TaskSetType &tasks, int index);
};

/**
 * @brief 
 * 
 * @tparam Schedul_Analysis 
 * @param tasks 
 * @param warmStart 
 * @param whetherPrint 
 * @param tol positive value, makes schedulability check more strict
 * @return true: system is schedulable
 * @return false: system is not schedulable
 */
template <class TaskType, template <typename> class Schedul_Analysis>
bool CheckSchedulability(const std::vector<TaskType> &tasks, VectorDynamic warmStart,
                         bool whetherPrint = false, double tol = 0)
{
    int N = tasks.size();
    for (int i = 0; i < N; i++)
    {
        double rta = Schedul_Analysis<TaskType>::RTA_Common_Warm(warmStart(i, 0), tasks, i);
        if (whetherPrint)
            cout << "response time for task " << i << " is " << rta << " and deadline is " << tasks[i].deadline << endl;
        if (rta + tol > min(tasks[i].deadline, tasks[i].period))
            return false;
    }
    if (whetherPrint)
        cout << endl;
    return true;
}

template <class TaskType, template <typename> class Schedul_Analysis>
bool CheckSchedulability(const std::vector<TaskType> &tasks, bool whetherPrint = false)
{
    VectorDynamic warmStart = GetParameterVD<double>(tasks, "executionTime");
    return CheckSchedulability<TaskType, Schedul_Analysis>(tasks, warmStart, whetherPrint);
}

template <class TaskType>
bool CheckSchedulabilityDirect(const std::vector<TaskType> &tasks, const VectorDynamic &rta)
{
    int N = tasks.size();
    for (int i = 0; i < N; i++)
    {
        if (rta(i, 0) > min(tasks[i].deadline, tasks[i].period))
            return false;
    }
    return true;
}

template <class TaskType, template <typename> class Schedul_Analysis>
VectorDynamic ResponseTimeOfTaskSet(const std::vector<TaskType> &tasks, const VectorDynamic &warmStart)
{
    int N = tasks.size();
    VectorDynamic res;
    res.resize(N, 1);

    if (debugMode == 1)
        cout << Color::blue << "RTA analysis (responseTime, deadline)" << Color::def << endl;
    for (int i = 0; i < N; i++)
    {
        res(i, 0) = Schedul_Analysis<TaskType>::RTA_Common_Warm(warmStart(i, 0), tasks, i);
        if (debugMode == 1)
            cout << res(i, 0) << ", " << tasks[i].deadline << endl;
        if (res(i, 0) > min(tasks[i].deadline, tasks[i].period))
        {
            if (debugMode == 1)
                cout << "The current task set is not schedulable!\n";
        }
    }
    return res;
}

template <class TaskType, template <typename> class Schedul_Analysis>
VectorDynamic ResponseTimeOfTaskSet(const std::vector<TaskType> &tasks)
{
    VectorDynamic warmStart = GetParameterVD<double>(tasks, "executionTime");
    return ResponseTimeOfTaskSet<TaskType, Schedul_Analysis>(tasks, warmStart);
}