#pragma once

#include "Tasks.h"
#include "Declaration.h"
/**
 * @brief All customized TaskSetType must inherit from TaskSetNormal in Tasks.h
 * 
 */

/**
 * @brief RTA_BASE encapsulate all the interafaces required
 *  by Energy_optimization.
 * All kinds of RTA should inherit 
 * from RTA_BASE and implement its virtual function
 * 
 */

template <class TaskSetType>
class RTA_BASE
{
public:
    /**
     * @brief 
     * 
     * @param tasks inverse priority ordered
     * @param index 
     * @return double 
     */
    static double RTA_Common_Warm(double beginTime, const TaskSetType &tasks, int index);
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
template <class TaskSetType, class Schedul_Analysis>
bool CheckSchedulability(const TaskSetType &tasks, VectorDynamic warmStart,
                         bool whetherPrint = false, double tol = 0)
{
    int N = tasks.tasks_.size();
    for (int i = 0; i < N; i++)
    {
        double rta = Schedul_Analysis::RTA_Common_Warm(warmStart(i, 0), tasks, i);
        if (whetherPrint)
            cout << "response time for task " << i << " is " << rta << " and deadline is " << tasks.tasks_[i].deadline << endl;
        if (rta + tol > min(tasks.tasks_[i].deadline, tasks.tasks_[i].period))
            return false;
    }
    if (whetherPrint)
        cout << endl;
    return true;
}

template <class TaskSetType, class Schedul_Analysis>
bool CheckSchedulability(const TaskSetType &tasks, bool whetherPrint = false)
{
    VectorDynamic warmStart = GetParameterVD<double>(tasks.tasks_, "executionTime");
    return CheckSchedulability<TaskSetType, Schedul_Analysis>(tasks, warmStart, whetherPrint);
}

template <class TaskSetType>
bool CheckSchedulabilityDirect(const TaskSetType &tasks, const VectorDynamic &rta)
{
    int N = tasks.tasks_.size();
    for (int i = 0; i < N; i++)
    {
        if (rta(i, 0) > min(tasks.tasks_[i].deadline, tasks.tasks_[i].period))
            return false;
    }
    return true;
}

template <class TaskSetType, class Schedul_Analysis>
VectorDynamic ResponseTimeOfTaskSet(const TaskSetType &tasks, const VectorDynamic &warmStart)
{
    int N = tasks.tasks_.size();
    VectorDynamic res;
    res.resize(N, 1);

    if (debugMode == 1)
        cout << Color::blue << "RTA analysis (responseTime, deadline)" << Color::def << endl;
    for (int i = 0; i < N; i++)
    {
        res(i, 0) = Schedul_Analysis::RTA_Common_Warm(warmStart(i, 0), tasks, i);
        if (debugMode == 1)
            cout << res(i, 0) << ", " << tasks.tasks_[i].deadline << endl;
        if (res(i, 0) > min(tasks.tasks_[i].deadline, tasks.tasks_[i].period))
        {
            if (debugMode == 1)
                cout << "The current task set is not schedulable!\n";
        }
    }
    return res;
}

template <class TaskSetType, class Schedul_Analysis>
VectorDynamic ResponseTimeOfTaskSet(const TaskSetType &tasks)
{
    VectorDynamic warmStart = GetParameterVD<double>(tasks, "executionTime");
    return ResponseTimeOfTaskSet<TaskSetType, Schedul_Analysis>(tasks, warmStart);
}