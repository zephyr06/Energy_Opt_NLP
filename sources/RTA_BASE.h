#pragma once

#include "Tasks.h"
#include "Declaration.h"

/**
 * @brief RTA_BASE encapsulate all the interafaces required
 *  by Energy_optimization.
 * All kinds of RTA should inherit 
 * from RTA_BASE and implement its virtual function
 * 
 */

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
    double RTA_Common(const TaskSet &tasks, int index);
    double RTA_Common_Warm(double beginTime, const TaskSet &tasks, int index);

    // double ResponseTimeAnalysisWarm(const double beginTime, const Task &taskCurr, const TaskSet &tasksHighPriority);
    // double ResponseTimeAnalysis(const Task &taskCurr, const TaskSet &tasksHighPriority);
};

template <class Schedul_Analysis>
bool CheckSchedulability(const TaskSet &taskSet, bool whetherPrint = false)
{
    int N = taskSet.size();
    for (int i = 0; i < N; i++)
    {
        TaskSet::const_iterator first = taskSet.begin();
        vector<Task>::const_iterator last = taskSet.begin() + i;
        TaskSet hpTasks(first, last);
        // double rta = Schedul_Analysis::ResponseTimeAnalysis(taskSet[i], hpTasks);
        double rta = Schedul_Analysis::RTA_Common(taskSet, i);
        if (whetherPrint)
            cout << "response time for task " << i << " is " << rta << " and deadline is " << taskSet[i].deadline << endl;
        if (rta > min(taskSet[i].deadline, taskSet[i].period))
            return false;
    }
    return true;
}

template <class Schedul_Analysis>
bool CheckSchedulability(const TaskSet &taskSet, VectorDynamic warmStart, bool whetherPrint = false)
{
    int N = taskSet.size();
    for (int i = 0; i < N; i++)
    {
        TaskSet::const_iterator first = taskSet.begin();
        vector<Task>::const_iterator last = taskSet.begin() + i;
        TaskSet hpTasks(first, last);
        // double rta = Schedul_Analysis::ResponseTimeAnalysisWarm(warmStart(i, 0), taskSet[i], hpTasks);
        double rta = Schedul_Analysis::RTA_Common_Warm(warmStart(i, 0), taskSet, i);
        if (whetherPrint)
            cout << "response time for task " << i << " is " << rta << " and deadline is " << taskSet[i].deadline << endl;
        if (rta > min(taskSet[i].deadline, taskSet[i].period))
            return false;
    }
    return true;
}

template <class Schedul_Analysis>
VectorDynamic ResponseTimeOfTaskSetHard(TaskSet &tasks)
{
    int N = tasks.size();
    VectorDynamic res;
    res.resize(N, 1);

    vector<Task> hpTasks;
    if (debugMode == 1)
        cout << "RTA analysis (responseTime, deadline)" << endl;
    for (int i = 0; i < N; i++)
    {
        // res(i, 0) = Schedul_Analysis::ResponseTimeAnalysis(tasks[i], hpTasks);
        res(i, 0) = Schedul_Analysis::RTA_Common(tasks, i);
        if (debugMode == 1)
            cout << res(i, 0) << ", " << tasks[i].deadline << endl;
        if (res(i, 0) > min(tasks[i].deadline, tasks[i].period))
        {
            if (debugMode == 1)
                cout << "The current task set is not schedulable!\n";
            res(0, 0) = -1;
            return res;
        }
        hpTasks.push_back(tasks[i]);
    }
    return res;
}